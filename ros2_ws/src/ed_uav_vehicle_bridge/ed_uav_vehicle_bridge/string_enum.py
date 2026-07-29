"""Python 3.10-compatible string-valued enum base."""

from enum import Enum


class StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value
