"""
JSON-RPC 2.0 message envelopes for the control plane.

Wire format is newline-delimited JSON (NDJSON): one JSON object per line,
terminated by a newline character. This is trivially streamable and debuggable
with ``netcat`` -- the team will use it that way.

Architecture note: this module lives in ``isaac_core.control`` (not in ``sim``)
so that both the server (inside Isaac's interpreter) and the client (a laptop
with no Isaac Sim at all) share identical message definitions with no extra
dependencies beyond pydantic.
"""

from enum import Enum
from typing import Any

from pydantic import BaseModel, ConfigDict, Field

from isaac_core.control.errors import (
    INTERNAL_ERROR_CODE,
    RpcError,
    error_from_code,
)


class Method(str, Enum):
    """
    Supported JSON-RPC method names.

    Using an enum ensures typos fail at import time rather than at runtime, and
    gives IDE completion.
    """

    GET_STATE = "get_state"
    GET_CAPABILITIES = "get_capabilities"
    GET_CONFIG = "get_config"
    SET_CONFIG = "set_config"
    ENABLE_FEATURE = "enable_feature"
    DISABLE_FEATURE = "disable_feature"
    LOAD_SCENE = "load_scene"
    RESET = "reset"
    PAUSE = "pause"
    RESUME = "resume"
    STEP = "step"
    CAPTURE_FRAME = "capture_frame"
    SET_POSE = "set_pose"
    PING = "ping"


class RpcRequest(BaseModel):
    """
    A JSON-RPC 2.0 request.

    The ``token`` field is an extension for authenticated non-loopback channels;
    it is stripped before dispatch and never forwarded to handlers.
    """

    model_config = ConfigDict(frozen=True)

    jsonrpc: str = "2.0"
    method: str
    params: dict[str, Any] | list[Any] | None = None
    id: int | str | None = Field(default=None)  # noqa: A003
    token: str | None = None


class RpcErrorData(BaseModel):
    """
    The ``error`` object inside a JSON-RPC 2.0 error response.

    Round-trips cleanly: server encodes from an ``RpcError`` exception, client
    decodes back to the same exception type.
    """

    model_config = ConfigDict(frozen=True)

    code: int
    message: str
    data: Any = None

    @classmethod
    def from_exception(cls, exc: RpcError) -> "RpcErrorData":
        """
        Build from a typed ``RpcError``.

        Args:
            exc: The exception to encode.

        Returns:
            An ``RpcErrorData`` ready for wire serialisation.

        """
        return cls(code=exc.code, message=exc.rpc_message, data=exc.data)

    @classmethod
    def from_unhandled(cls, exc: BaseException) -> "RpcErrorData":
        """
        Wrap an unexpected exception as an internal error.

        Args:
            exc: The unexpected exception.

        Returns:
            An ``RpcErrorData`` with the internal error code and exception repr.

        """
        return cls(code=INTERNAL_ERROR_CODE, message=repr(exc))

    def to_exception(self) -> RpcError:
        """
        Reconstruct the typed exception on the client side.

        Returns:
            The appropriate ``RpcError`` subclass.

        """
        return error_from_code(self.code, self.message, self.data)


class RpcResponse(BaseModel):
    """
    A JSON-RPC 2.0 response (success or error, never both).

    Construction helpers :meth:`success` and :meth:`error` enforce the mutual
    exclusion invariant.
    """

    model_config = ConfigDict(frozen=True)

    jsonrpc: str = "2.0"
    result: Any = None
    error: RpcErrorData | None = None
    id: int | str | None = Field(default=None)  # noqa: A003

    @classmethod
    def success(cls, result: Any, request_id: int | str | None) -> "RpcResponse":  # noqa: ANN401
        """
        Build a success response.

        Args:
            result: The handler's return value.
            request_id: Echoed from the request.

        Returns:
            A valid success response.

        """
        return cls(result=result, id=request_id)

    @classmethod
    def error_response(cls, err: RpcErrorData, request_id: int | str | None = None) -> "RpcResponse":
        """
        Build an error response.

        Args:
            err: The structured error.
            request_id: Echoed from the request, or ``None`` if unparseable.

        Returns:
            A valid error response.

        """
        return cls(error=err, id=request_id)


def encode(msg: RpcRequest | RpcResponse) -> bytes:
    """
    Serialise a message to a single NDJSON line (UTF-8 bytes ending with newline).

    Args:
        msg: The pydantic model to serialise.

    Returns:
        UTF-8 encoded JSON followed by a newline byte.

    """
    line: str = msg.model_dump_json(exclude_none=True)
    return line.encode("utf-8") + b"\n"


def decode_request(line: bytes) -> RpcRequest:
    """
    Parse one NDJSON line into an ``RpcRequest``.

    Args:
        line: Raw bytes (with or without trailing newline).

    Returns:
        A validated request model.

    Raises:
        ValueError: If JSON is malformed or does not match the schema.

    """
    result: RpcRequest = RpcRequest.model_validate_json(line.strip())
    return result


def decode_response(line: bytes) -> RpcResponse:
    """
    Parse one NDJSON line into an ``RpcResponse``.

    Args:
        line: Raw bytes (with or without trailing newline).

    Returns:
        A validated response model.

    Raises:
        ValueError: If JSON is malformed or does not match the schema.

    """
    result: RpcResponse = RpcResponse.model_validate_json(line.strip())
    return result


__all__ = [
    "Method",
    "RpcErrorData",
    "RpcRequest",
    "RpcResponse",
    "decode_request",
    "decode_response",
    "encode",
]
