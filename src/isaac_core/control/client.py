"""Client for the JSON-RPC control plane.

Connects to a running :class:`~isaac_core.control.server.ControlServer`, sends
requests, and returns typed results or raises typed exceptions from
:mod:`isaac_core.control.errors`.

The :meth:`ControlClient.wait_until_ready` method is the documented replacement
for grepping logs and sleeping to detect readiness: it polls until the port
accepts a connection and a ``ping`` succeeds, so the port becoming connectable
IS the readiness signal.
"""

import socket
import time
from typing import Any

from isaac_core.contracts.ports import DEFAULT_CONTROL_PLANE_PORT
from isaac_core.control.errors import RpcError
from isaac_core.control.messages import (
    Method,
    RpcRequest,
    decode_response,
    encode,
)

# Default socket timeout for a call. Generous because several methods legitimately span multiple
# render frames -- `capture_frame` resizes, settles and waits for an async file write, and `step`
# runs N frames -- and a short timeout reported those as failures while they were still working.
DEFAULT_CALL_TIMEOUT_S: float = 60.0


class ControlClient:
    """Synchronous JSON-RPC 2.0 client over TCP with newline-delimited messages.

    Usage::

        client = ControlClient("127.0.0.1", 8760)
        client.connect()
        result = client.call("ping")
        client.close()

    Or as a context manager::

        with ControlClient("127.0.0.1", 8760) as client:
            result = client.call("ping")

    """

    def __init__(
        self, host: str = "127.0.0.1", port: int = DEFAULT_CONTROL_PLANE_PORT, token: str | None = None
    ) -> None:
        """Initialise the client.

        Args:
            host: Server host.
            port: Server port.
            token: Authentication token for non-loopback servers.

        """
        self._host = host
        self._port = port
        self._token = token
        self._sock: socket.socket | None = None
        self._buffer: bytes = b""
        self._request_id: int = 0

    @property
    def host(self) -> str:
        """Return the target host."""
        return self._host

    @property
    def port(self) -> int:
        """Return the target port."""
        return self._port

    def connect(self, timeout: float = DEFAULT_CALL_TIMEOUT_S) -> None:
        """Establish TCP connection to the server.

        Args:
            timeout: Socket timeout in seconds.

        Raises:
            ConnectionRefusedError: If the server is not listening.
            OSError: On other socket failures.

        """
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(timeout)
        self._sock.connect((self._host, self._port))
        self._buffer = b""

    def close(self) -> None:
        """Close the connection."""
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self._buffer = b""

    def call(self, method: str, params: dict[str, Any] | list[Any] | None = None) -> Any:
        """Send a JSON-RPC request and return the result.

        Args:
            method: The method name.
            params: Optional parameters.

        Returns:
            The ``result`` field from the response.

        Raises:
            RpcError: (or a subclass) if the server returns an error response.
            ConnectionError: If the socket is not connected.
            TimeoutError: If the server does not respond within the socket timeout.

        """
        if self._sock is None:
            msg = "not connected — call connect() first"
            raise ConnectionError(msg)

        self._request_id += 1
        request = RpcRequest(
            method=method,
            params=params,
            id=self._request_id,
            token=self._token,
        )
        self._sock.sendall(encode(request))

        # Read until we have a complete line.
        while b"\n" not in self._buffer:
            chunk = self._sock.recv(65536)
            if not chunk:
                msg = "connection closed by server"
                raise ConnectionError(msg)
            self._buffer += chunk

        line, self._buffer = self._buffer.split(b"\n", 1)
        response = decode_response(line)

        if response.error is not None:
            raise response.error.to_exception()

        return response.result

    def ping(self) -> Any:
        """Send a ping request.

        Returns:
            The server's pong result.

        """
        return self.call(Method.PING.value)

    def _stage_ready(self) -> bool:
        """Report whether the simulator has composed its stage.

        Tolerates an older simulator that does not publish the field, treating a missing ``ready``
        as ready so a newer client cannot hang forever against one.

        Returns:
            ``True`` when the stage is composed, or the field is absent.

        """
        try:
            state = self.call(Method.GET_STATE.value)
        except RpcError:
            # A server that does not implement get_state cannot report readiness, so treat it as
            # ready rather than hanging. Keeps a newer client usable against an older simulator,
            # and keeps minimal test servers working.
            return True
        if not isinstance(state, dict) or "ready" not in state:
            return True
        return bool(state["ready"])

    def wait_until_ready(
        self,
        timeout_s: float = 30.0,
        poll_interval: float = 0.1,
        *,
        require_stage: bool = True,
    ) -> None:
        """Block until the simulator is actually usable, not merely listening.

        A connectable port is **not** readiness. The control plane starts listening before the
        stage is composed, so commands sent in that window are accepted and then silently lost --
        the graphs that would act on them do not exist yet. This was observed for real: a
        ``set_pose`` issued right after the port opened left the camera motionless, with no error
        anywhere. So by default this also waits for ``get_state`` to report ``ready``.

        Args:
            timeout_s: Maximum total time to wait.
            poll_interval: Seconds between attempts.
            require_stage: Wait for the stage to be composed as well as the port to answer. Pass
                ``False`` only to talk to a simulator mid-startup deliberately.

        Raises:
            TimeoutError: If the server does not become ready within ``timeout_s``.

        """
        deadline = time.monotonic() + timeout_s
        last_error: BaseException | None = None

        while time.monotonic() < deadline:
            try:
                # A generous socket timeout, not the poll interval: some methods legitimately take
                # seconds (a capture spans several frames, `step` runs N of them), and a short
                # socket timeout made them fail as TimeoutError while the work was still going.
                self.connect(timeout=max(DEFAULT_CALL_TIMEOUT_S, poll_interval))
                self.ping()
                if not require_stage or self._stage_ready():
                    return
                last_error = None
            except (OSError, ConnectionError, RpcError) as exc:
                last_error = exc
            self.close()
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(poll_interval, remaining))

        msg = f"server at {self._host}:{self._port} not ready within {timeout_s}s"
        if last_error is not None:
            msg = f"{msg} (last error: {last_error!r})"
        raise TimeoutError(msg)

    def __enter__(self) -> "ControlClient":
        """Enter context — connect."""
        self.connect()
        return self

    def __exit__(self, *_args: object) -> None:
        """Exit context — close."""
        self.close()


__all__ = [
    "ControlClient",
]
