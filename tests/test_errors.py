"""TransportError — the typed, structured failure shape."""

from castor_hal.errors import TransportError, TransportErrorCode


def test_str_is_code_prefixed():
    e = TransportError(TransportErrorCode.OUT_OF_RANGE, "elbow past limit")
    assert str(e) == "out_of_range: elbow past limit"


def test_as_dict_minimal():
    e = TransportError(TransportErrorCode.TIMEOUT, "no ack")
    assert e.as_dict() == {"error_code": "timeout", "error_message": "no ack"}


def test_as_dict_with_detail():
    e = TransportError(
        TransportErrorCode.OUT_OF_RANGE, "bad value",
        detail={"joint": "elbow_flex", "value": 3.1, "limit": [-1.8, 1.8]},
    )
    d = e.as_dict()
    assert d["error_code"] == "out_of_range"
    assert d["error_detail"]["joint"] == "elbow_flex"


def test_codes_are_strings_for_json():
    assert TransportErrorCode.ESTOPPED.value == "estopped"
    assert isinstance(TransportErrorCode.ESTOPPED.value, str)


def test_is_an_exception():
    try:
        raise TransportError(TransportErrorCode.IO_ERROR, "wire down")
    except Exception as e:  # noqa: BLE001
        assert isinstance(e, TransportError)
        assert e.code is TransportErrorCode.IO_ERROR
