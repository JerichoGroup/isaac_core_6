"""Tests for JSON-RPC message encoding, decoding and the Method enum."""

import json

import pytest

from isaac_core.control.errors import INTERNAL_ERROR_CODE, InternalError, ParseError
from isaac_core.control.messages import (
    Method,
    RpcErrorData,
    RpcRequest,
    RpcResponse,
    decode_request,
    decode_response,
    encode,
)


def test_method_enum_contains_all_required_names() -> None:
    expected = {
        "get_state",
        "get_capabilities",
        "get_config",
        "set_config",
        "enable_feature",
        "disable_feature",
        "load_scene",
        "reset",
        "pause",
        "resume",
        "step",
        "capture_frame",
        "set_pose",
        "ping",
        "get_pose",
    }
    actual = {m.value for m in Method}
    assert actual == expected


def test_method_enum_values_are_their_string_names() -> None:
    # Enum values should be usable directly as JSON-RPC method strings.
    for member in Method:
        assert member.value == member.name.lower()


def test_encode_request_produces_ndjson() -> None:
    req = RpcRequest(method="ping", id=1)
    raw = encode(req)
    assert raw.endswith(b"\n")
    parsed = json.loads(raw)
    assert parsed["method"] == "ping"
    assert parsed["jsonrpc"] == "2.0"
    assert parsed["id"] == 1


def test_encode_request_omits_none_fields() -> None:
    req = RpcRequest(method="ping", id=1)
    raw = encode(req)
    parsed = json.loads(raw)
    assert "params" not in parsed
    assert "token" not in parsed


def test_encode_request_includes_params_when_set() -> None:
    req = RpcRequest(method="set_config", params={"key": "value"}, id=2)
    raw = encode(req)
    parsed = json.loads(raw)
    assert parsed["params"] == {"key": "value"}


def test_encode_request_includes_token_when_set() -> None:
    req = RpcRequest(method="ping", id=3, token="secret")
    raw = encode(req)
    parsed = json.loads(raw)
    assert parsed["token"] == "secret"


def test_decode_request_round_trips() -> None:
    req = RpcRequest(method="get_state", params={"verbose": True}, id=42)
    raw = encode(req)
    decoded = decode_request(raw)
    assert decoded.method == "get_state"
    assert decoded.params == {"verbose": True}
    assert decoded.id == 42


def test_decode_request_strips_trailing_whitespace() -> None:
    raw = b'{"jsonrpc":"2.0","method":"ping","id":1}  \n'
    decoded = decode_request(raw)
    assert decoded.method == "ping"


def test_decode_request_rejects_garbage() -> None:
    with pytest.raises(ValueError):
        decode_request(b"not json at all\n")


def test_encode_success_response() -> None:
    resp = RpcResponse.success(result={"status": "running"}, request_id=7)
    raw = encode(resp)
    parsed = json.loads(raw)
    assert parsed["result"] == {"status": "running"}
    assert parsed["id"] == 7
    assert "error" not in parsed


def test_encode_error_response() -> None:
    err = RpcErrorData.from_exception(ParseError("bad json"))
    resp = RpcResponse.error_response(err, request_id=8)
    raw = encode(resp)
    parsed = json.loads(raw)
    assert parsed["error"]["code"] == -32700
    assert parsed["error"]["message"] == "bad json"
    assert parsed["id"] == 8
    assert "result" not in parsed


def test_decode_response_success_round_trip() -> None:
    resp = RpcResponse.success(result="pong", request_id=10)
    raw = encode(resp)
    decoded = decode_response(raw)
    assert decoded.result == "pong"
    assert decoded.error is None
    assert decoded.id == 10


def test_decode_response_error_round_trip() -> None:
    err = RpcErrorData(code=-32601, message="Method not found")
    resp = RpcResponse.error_response(err, request_id=11)
    raw = encode(resp)
    decoded = decode_response(raw)
    assert decoded.error is not None
    assert decoded.error.code == -32601


def test_rpc_error_data_from_unhandled_exception() -> None:
    exc = RuntimeError("kaboom")
    err = RpcErrorData.from_unhandled(exc)
    assert err.code == INTERNAL_ERROR_CODE
    assert "kaboom" in err.message


def test_rpc_error_data_to_exception_reconstructs_type() -> None:
    err = RpcErrorData.from_exception(InternalError("something broke", data={"detail": 1}))
    reconstructed = err.to_exception()
    assert isinstance(reconstructed, InternalError)
    assert reconstructed.rpc_message == "something broke"
    assert reconstructed.data == {"detail": 1}


def test_request_with_list_params() -> None:
    req = RpcRequest(method="step", params=[5], id=12)
    raw = encode(req)
    decoded = decode_request(raw)
    assert decoded.params == [5]


def test_request_with_string_id() -> None:
    req = RpcRequest(method="ping", id="abc-123")
    raw = encode(req)
    decoded = decode_request(raw)
    assert decoded.id == "abc-123"


def test_response_with_null_result_still_encodes() -> None:
    # A method that returns nothing (like reset) should still produce a valid response.
    resp = RpcResponse.success(result=None, request_id=13)
    raw = encode(resp)
    parsed = json.loads(raw)
    # result=None is excluded by exclude_none, but the response is still valid.
    assert parsed["id"] == 13


def test_rpc_error_data_preserves_data_field() -> None:
    err = RpcErrorData(code=-32602, message="bad param", data={"param": "x", "reason": "missing"})
    resp = RpcResponse.error_response(err, request_id=14)
    raw = encode(resp)
    decoded = decode_response(raw)
    assert decoded.error is not None
    assert decoded.error.data == {"param": "x", "reason": "missing"}
