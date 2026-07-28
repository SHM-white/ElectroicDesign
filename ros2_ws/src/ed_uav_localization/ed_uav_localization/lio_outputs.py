from copy import deepcopy
from typing import Protocol, TypeVar


class Header(Protocol):
    frame_id: str


class HeaderMessage(Protocol):
    header: Header


class PathMessage(HeaderMessage, Protocol):
    poses: list[HeaderMessage]


Message = TypeVar("Message", bound=HeaderMessage)
PathLike = TypeVar("PathLike", bound=PathMessage)


def canonicalize_cloud(message: Message) -> Message:
    canonical = deepcopy(message)
    canonical.header.frame_id = "odom"
    return canonical


def canonicalize_path(path: PathLike) -> PathLike:
    canonical = deepcopy(path)
    canonical.header.frame_id = "odom"
    for pose in canonical.poses:
        pose.header.frame_id = "odom"
    return canonical
