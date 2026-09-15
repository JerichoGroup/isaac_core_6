"""Typed exceptions for JSON-RPC error responses.

Each exception carries the standard JSON-RPC error code and maps cleanly to the
wire format defined in ``messages.RpcErrorData``. Round-trip fidelity is tested:
raise on the server → encode → decode on the client → same exception type raised.

Error codes follow the JSON-RPC 2.0 specification (§5.1) plus one application-
defined code for authentication failures.
"""

from typing import Any, Final

# Standard JSON-RPC 2.0 error codes.
PARSE_ERROR_CODE: Final = -32700
INVALID_REQUEST_CODE: Final = -32600
METHOD_NOT_FOUND_CODE: Final = -32601
INVALID_PARAMS_CODE: Final = -32602
INTERNAL_ERROR_CODE: Final = -32603

# Application-defined: authentication/authorisation failure.
AUTH_ERROR_CODE: Final = -32000


class RpcError(Exception):
    """Base for all JSON-RPC errors that travel over the wire.

    Subclasses define the ``code`` class variable. Handlers may raise any subclass
    and the server will encode it into a proper JSON-RPC error response.
    """

    code: int = INTERNAL_ERROR_CODE

    def __init__(self, message: str = "", data: Any = None) -> None:
        """Initialise an RPC error.

        Args:
            message: Human-readable error description.
            data: Optional structured payload for debugging.

        """
        super().__init__(message)
        self.rpc_message: str = message
        self.data: Any = data


class ParseError(RpcError):
    """Malformed JSON received."""

    code: int = PARSE_ERROR_CODE

    def __init__(self, message: str = "Parse error", data: Any = None) -> None:
        """Initialise a parse error.

        Args:
            message: Description.
            data: Optional detail.

        """
        super().__init__(message, data)


class InvalidRequestError(RpcError):
    """Valid JSON but not a conforming JSON-RPC request."""

    code: int = INVALID_REQUEST_CODE

    def __init__(self, message: str = "Invalid request", data: Any = None) -> None:
        """Initialise an invalid-request error.

        Args:
            message: Description.
            data: Optional detail.

        """
        super().__init__(message, data)


class MethodNotFoundError(RpcError):
    """Requested method does not exist."""

    code: int = METHOD_NOT_FOUND_CODE

    def __init__(self, message: str = "Method not found", data: Any = None) -> None:
        """Initialise a method-not-found error.

        Args:
            message: Description.
            data: Optional detail.

        """
        super().__init__(message, data)


class InvalidParamsError(RpcError):
    """Method exists but parameters are wrong."""

    code: int = INVALID_PARAMS_CODE

    def __init__(self, message: str = "Invalid params", data: Any = None) -> None:
        """Initialise an invalid-params error.

        Args:
            message: Description.
            data: Optional detail.

        """
        super().__init__(message, data)


class InternalError(RpcError):
    """Unhandled exception inside a handler."""

    code: int = INTERNAL_ERROR_CODE

    def __init__(self, message: str = "Internal error", data: Any = None) -> None:
        """Initialise an internal error.

        Args:
            message: Description.
            data: Optional detail.

        """
        super().__init__(message, data)


class AuthError(RpcError):
    """Token missing or invalid on a non-loopback connection."""

    code: int = AUTH_ERROR_CODE

    def __init__(self, message: str = "Authentication required", data: Any = None) -> None:
        """Initialise an authentication error.

        Args:
            message: Description.
            data: Optional detail.

        """
        super().__init__(message, data)


# Map error codes back to exception classes for client-side reconstruction.
_CODE_TO_CLASS: Final[dict[int, type[RpcError]]] = {
    PARSE_ERROR_CODE: ParseError,
    INVALID_REQUEST_CODE: InvalidRequestError,
    METHOD_NOT_FOUND_CODE: MethodNotFoundError,
    INVALID_PARAMS_CODE: InvalidParamsError,
    INTERNAL_ERROR_CODE: InternalError,
    AUTH_ERROR_CODE: AuthError,
}


def error_from_code(code: int, message: str, data: Any = None) -> RpcError:
    """Reconstruct a typed exception from a wire error code.

    Args:
        code: JSON-RPC error code.
        message: Human-readable description.
        data: Optional structured payload.

    Returns:
        The most specific ``RpcError`` subclass matching the code.

    """
    cls = _CODE_TO_CLASS.get(code, RpcError)
    return cls(message, data)


__all__ = [
    "AUTH_ERROR_CODE",
    "AuthError",
    "INTERNAL_ERROR_CODE",
    "INVALID_PARAMS_CODE",
    "INVALID_REQUEST_CODE",
    "InternalError",
    "InvalidParamsError",
    "InvalidRequestError",
    "METHOD_NOT_FOUND_CODE",
    "MethodNotFoundError",
    "PARSE_ERROR_CODE",
    "ParseError",
    "RpcError",
    "error_from_code",
]
