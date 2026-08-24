"""
TCP server for the JSON-RPC control plane.

Binds a TCP socket, accepts connections, reads newline-delimited JSON-RPC
requests, dispatches to registered handlers, and writes responses. Designed to
run inside Isaac Sim's interpreter as a background thread, while remaining
testable in isolation (no Isaac imports).

Security:
- Binds loopback by default.
- If the configured host is NOT loopback, a token is REQUIRED on every request.
- Path arguments (e.g. for ``capture_frame``) are confined under the configured
  ``output_root`` via :func:`confine_path`, preventing directory traversal.

Architecture note: handlers are REGISTERED via :meth:`ControlServer.register`,
not hardcoded, so ``sim`` can supply Isaac-backed implementations later without
this module importing Isaac.
"""

import json
import logging
from pathlib import Path
import selectors
import socket
import threading
from typing import Any, Callable

from isaac_core.config.schema import ControlPlaneConfig
from isaac_core.control.errors import (
    AuthError,
    MethodNotFoundError,
    ParseError,
    RpcError,
)
from isaac_core.control.messages import (
    RpcErrorData,
    RpcResponse,
    decode_request,
    encode,
)

logger = logging.getLogger(__name__)

# Hosts considered loopback — token enforcement is skipped for these.
_LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})

# Handler signature: receives params dict/list/None, returns Any result.
Handler = Callable[[dict[str, Any] | list[Any] | None], Any]


def confine_path(requested: str, output_root: Path) -> Path:
    """
    Resolve a requested path and verify it lies under ``output_root``.

    Rejects absolute paths not under the root, ``..`` traversal, and any resolved
    path that escapes. This is a security boundary: getting it wrong is remote
    file write / read outside the allowed tree.

    Args:
        requested: The path string from the client.
        output_root: The configured root directory all outputs must stay within.

    Returns:
        The resolved, confined ``Path``.

    Raises:
        PermissionError: If the resolved path escapes ``output_root``.

    """
    root = output_root.expanduser().resolve()
    # Treat the requested path as relative to output_root.
    # If it's absolute, check directly; otherwise join.
    candidate = Path(requested)
    if candidate.is_absolute():
        resolved = candidate.resolve()
    else:
        resolved = (root / candidate).resolve()

    # The resolved path must start with the root.
    try:
        resolved.relative_to(root)
    except ValueError:
        msg = f"path {requested!r} resolves to {resolved} which is outside output_root {root}"
        raise PermissionError(msg) from None
    return resolved


class ControlServer:
    """
    JSON-RPC 2.0 control server over TCP with newline-delimited messages.

    Lifecycle::

        server = ControlServer(config)
        server.register("ping", lambda params: "pong")
        server.start()   # spawns a background daemon thread
        ...
        server.stop()    # clean shutdown, joins the thread

    """

    def __init__(
        self,
        config: ControlPlaneConfig | None = None,
        *,
        bind_port: int | None = None,
    ) -> None:
        """
        Initialise the server.

        Args:
            config: Control plane settings. Defaults to ``ControlPlaneConfig()``
                (loopback, no token required).
            bind_port: Override the port to bind (useful for tests that need
                ephemeral port 0). If ``None``, uses ``config.port``.

        """
        self._config = config or ControlPlaneConfig()
        self._bind_port = bind_port if bind_port is not None else self._config.port
        self._handlers: dict[str, Handler] = {}
        self._server_socket: socket.socket | None = None
        self._selector: selectors.DefaultSelector | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._bound_port: int | None = None

    @property
    def port(self) -> int | None:
        """
        Return the port the server is actually bound to.

        Useful when binding port 0 (ephemeral) for tests.
        """
        return self._bound_port

    @property
    def host(self) -> str:
        """Return the configured host."""
        return self._config.host

    def register(self, method: str, handler: Handler) -> None:
        """
        Register a handler for a JSON-RPC method.

        Args:
            method: The method name (should be one of :class:`~messages.Method`).
            handler: Callable receiving ``params`` and returning a result.

        """
        self._handlers[method] = handler

    def start(self) -> None:
        """
        Bind the socket and start serving in a background daemon thread.

        The server is ready (accepting connections) by the time this returns.
        """
        self._stop_event.clear()
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self._config.host, self._bind_port))
        self._bound_port = self._server_socket.getsockname()[1]
        self._server_socket.listen(16)
        self._server_socket.setblocking(False)

        self._selector = selectors.DefaultSelector()
        self._selector.register(self._server_socket, selectors.EVENT_READ)

        self._thread = threading.Thread(
            target=self._serve_loop,
            name="isaac-core-control-server",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        """
        Signal the server to stop and wait for the thread to finish.

        Args:
            timeout: Maximum seconds to wait for the thread to join.

        """
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            self._thread = None
        if self._selector is not None:
            self._selector.close()
            self._selector = None
        if self._server_socket is not None:
            self._server_socket.close()
            self._server_socket = None
        self._bound_port = None

    def _serve_loop(self) -> None:
        """Select loop: accept connections and read from clients."""
        assert self._selector is not None  # noqa: S101
        assert self._server_socket is not None  # noqa: S101

        try:
            while not self._stop_event.is_set():
                events = self._selector.select(timeout=0.1)
                for key, _ in events:
                    if key.fileobj is self._server_socket:
                        self._accept_connection()
                    else:
                        self._handle_client_data(key)
        finally:
            # Close all registered client sockets.
            if self._selector is not None:
                for key in list(self._selector.get_map().values()):
                    if key.fileobj is not self._server_socket:
                        sock = key.fileobj
                        self._selector.unregister(sock)
                        if isinstance(sock, socket.socket):
                            sock.close()

    def _accept_connection(self) -> None:
        """Accept a new client and register it with the selector."""
        assert self._server_socket is not None  # noqa: S101
        assert self._selector is not None  # noqa: S101
        conn, _addr = self._server_socket.accept()
        conn.setblocking(False)
        self._selector.register(conn, selectors.EVENT_READ, data=b"")

    def _handle_client_data(self, key: selectors.SelectorKey) -> None:
        """Read available data from a client, processing complete lines."""
        assert self._selector is not None  # noqa: S101
        sock: socket.socket = key.fileobj  # type: ignore[assignment]
        try:
            data = sock.recv(65536)
        except (ConnectionResetError, OSError):
            data = b""

        if not data:
            self._selector.unregister(sock)
            sock.close()
            return

        buffer: bytes = key.data + data

        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            response = self._dispatch(line)
            try:
                sock.sendall(encode(response))
            except (BrokenPipeError, OSError):
                self._selector.unregister(sock)
                sock.close()
                return

        # Update the buffer for next read.
        self._selector.modify(sock, selectors.EVENT_READ, data=buffer)

    def _dispatch(self, line: bytes) -> RpcResponse:
        """Parse a request line and dispatch to the handler."""
        # Parse.
        try:
            request = decode_request(line)
        except (ValueError, json.JSONDecodeError) as exc:
            err = RpcErrorData.from_exception(ParseError(str(exc)))
            return RpcResponse.error_response(err)

        request_id = request.id

        # Authenticate if non-loopback.
        if self._config.host not in _LOOPBACK_HOSTS:
            if not request.token or request.token != self._config.token:
                err = RpcErrorData.from_exception(AuthError("invalid or missing token"))
                return RpcResponse.error_response(err, request_id)

        # Look up handler.
        handler = self._handlers.get(request.method)
        if handler is None:
            err = RpcErrorData.from_exception(MethodNotFoundError(f"method {request.method!r} is not registered"))
            return RpcResponse.error_response(err, request_id)

        # Dispatch.
        try:
            result = handler(request.params)
        except RpcError as exc:
            err = RpcErrorData.from_exception(exc)
            return RpcResponse.error_response(err, request_id)
        except Exception as exc:  # noqa: BLE001
            logger.exception("unhandled exception in handler for %r", request.method)
            err = RpcErrorData.from_unhandled(exc)
            return RpcResponse.error_response(err, request_id)

        return RpcResponse.success(result, request_id)


__all__ = [
    "ControlServer",
    "Handler",
    "confine_path",
]
