import threading

import pytest

from mavforge.domain.errors import ConnectionFailedError
from mavforge.transport.dialect import Message, mavlink
from mavforge.transport.router import MessageRouter


def _hb() -> Message:
    return mavlink.MAVLink_heartbeat_message(2, 3, 0, 0, 3, 3)


def _text() -> Message:
    return mavlink.MAVLink_statustext_message(6, b"hello", 0, 0)


def test_concurrent_subscribers_each_receive_matching_messages() -> None:
    """Unlike pymavlink's recv_match, one waiter must not consume another's messages."""
    router = MessageRouter()
    hb_sub = router.subscribe(lambda m: m.get_type() == "HEARTBEAT")
    all_sub = router.subscribe(lambda m: True)
    router.dispatch(_text())
    router.dispatch(_hb())
    hb = hb_sub.get(0.1)
    assert hb is not None
    assert hb.get_type() == "HEARTBEAT"
    assert hb_sub.get(0.01) is None
    first, second = all_sub.get(0.1), all_sub.get(0.1)
    assert first is not None
    assert second is not None
    assert [first.get_type(), second.get_type()] == ["STATUSTEXT", "HEARTBEAT"]


def test_unsubscribed_subscription_receives_nothing() -> None:
    router = MessageRouter()
    with router.subscribe(lambda m: True) as sub:
        pass
    router.dispatch(_hb())
    assert sub.get(0.01) is None


def test_full_buffer_drops_oldest_and_counts() -> None:
    router = MessageRouter()
    sub = router.subscribe(lambda m: True, maxsize=2)
    for _ in range(5):
        router.dispatch(_hb())
    assert sub.dropped == 3


def test_close_wakes_blocked_waiters_with_connection_error() -> None:
    router = MessageRouter()
    sub = router.subscribe(lambda m: True)
    errors: list[Exception] = []

    def wait() -> None:
        try:
            sub.get(5)
        except ConnectionFailedError as exc:
            errors.append(exc)

    t = threading.Thread(target=wait)
    t.start()
    router.close("link error: test")
    t.join(2)
    assert len(errors) == 1
    with pytest.raises(ConnectionFailedError):
        router.subscribe(lambda m: True).get(0.01)
