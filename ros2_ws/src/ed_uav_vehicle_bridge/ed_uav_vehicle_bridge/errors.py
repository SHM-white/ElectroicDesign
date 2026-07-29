"""Typed boundary failures for UDP v1."""

from dataclasses import dataclass
from enum import unique

from .string_enum import StringEnum


@unique
class ProtocolErrorCode(StringEnum):
    DATAGRAM_TOO_SHORT = "DATAGRAM_TOO_SHORT"
    DATAGRAM_TOO_LARGE = "DATAGRAM_TOO_LARGE"
    BAD_MAGIC = "BAD_MAGIC"
    BAD_VERSION = "BAD_VERSION"
    BAD_MESSAGE_TYPE = "BAD_MESSAGE_TYPE"
    BAD_LENGTH = "BAD_LENGTH"
    BAD_SENDER_ID = "BAD_SENDER_ID"
    BAD_BOOT_EPOCH = "BAD_BOOT_EPOCH"
    BAD_HMAC = "BAD_HMAC"
    BAD_CRC = "BAD_CRC"
    BAD_PAYLOAD = "BAD_PAYLOAD"
    KEY_TOO_SHORT = "KEY_TOO_SHORT"
    SOURCE_MISMATCH = "SOURCE_MISMATCH"
    MESSAGE_TYPE_FORBIDDEN = "MESSAGE_TYPE_FORBIDDEN"
    REPLAY = "REPLAY"
    REORDERED = "REORDERED"
    SEQUENCE_GAP = "SEQUENCE_GAP"
    RETIRED_BOOT_EPOCH = "RETIRED_BOOT_EPOCH"
    INVALID_ROUTE_ORDER = "INVALID_ROUTE_ORDER"
    START_EVENT_REPEATED = "START_EVENT_REPEATED"


@dataclass(frozen=True, slots=True)
class ProtocolError(Exception):
    code: ProtocolErrorCode
    detail: str

    def __str__(self) -> str:
        return f"{self.code}: {self.detail}"


@dataclass(frozen=True, slots=True)
class BridgeConfigError(Exception):
    field: str
    detail: str

    def __str__(self) -> str:
        return f"invalid {self.field}: {self.detail}"


@dataclass(frozen=True, slots=True)
class SocketClosedError(Exception):
    operation: str

    def __str__(self) -> str:
        return f"UDP socket is closed during {self.operation}"
