from __future__ import annotations

import os
import pty
import selectors
import subprocess
import sys
from contextlib import ExitStack
from pathlib import Path
from typing import Final

import pytest

PACKAGE_ROOT: Final = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from ed_uav_fcu_bridge.serial_port import ExclusiveSerialPort

OWNERSHIP_PROBE: Final = """\
from pathlib import Path
import sys
from ed_uav_fcu_bridge.serial_port import ExclusiveSerialPort, SerialOpenError, SerialOwnershipError
port = ExclusiveSerialPort(sys.argv[1], lock_dir=Path(sys.argv[2]))
try:
    port.open()
except SerialOwnershipError:
    print('SerialOwnershipError')
    raise SystemExit(3)
except SerialOpenError:
    print('SerialOpenError', file=sys.stderr)
    raise SystemExit(4)
else:
    port.close()
    raise SystemExit(0)"""


@pytest.mark.parametrize(
    "owner_uses_alias",
    [False, True],
    ids=["canonical-owner-alias-contender", "alias-owner-canonical-contender"],
)
def test_serial_alias_contender_reports_ownership(
    tmp_path: Path,
    *,
    owner_uses_alias: bool,
) -> None:
    # Given: one process owns a PTY through either its canonical path or symlink alias.
    with ExitStack() as resources:
        master_fd, slave_fd = pty.openpty()
        resources.callback(os.close, master_fd)
        resources.callback(os.close, slave_fd)
        canonical_path = Path(os.ttyname(slave_fd))
        alias_path = tmp_path / "fcu-serial-alias"
        alias_path.symlink_to(canonical_path)
        resources.callback(alias_path.unlink, missing_ok=True)
        owner_path = alias_path if owner_uses_alias else canonical_path
        contender_path = canonical_path if owner_uses_alias else alias_path
        owner = ExclusiveSerialPort(str(owner_path), lock_dir=tmp_path)
        owner.open()
        resources.callback(owner.close)
        environment = os.environ.copy()
        existing_pythonpath = environment.get("PYTHONPATH", "")
        environment["PYTHONPATH"] = (
            f"{PACKAGE_ROOT}{os.pathsep}{existing_pythonpath}"
            if existing_pythonpath
            else str(PACKAGE_ROOT)
        )

        # When: a separate process requests the same PTY through the other pathname.
        contender = subprocess.run(
            [sys.executable, "-c", OWNERSHIP_PROBE, str(contender_path), str(tmp_path)],
            capture_output=True,
            text=True,
            env=environment,
            check=False,
            timeout=2.0,
        )

        # Then: endpoint identity, not pathname spelling, determines ownership.
        assert contender.returncode == 3, contender.stderr
        assert contender.stdout.strip() == "SerialOwnershipError"


def test_writer_opened_before_exclusive_claim_remains_writable(tmp_path: Path) -> None:
    # Given: a raw PTY writer already exists before the bridge claims TIOCEXCL.
    with ExitStack() as resources:
        master_fd, slave_fd = pty.openpty()
        resources.callback(os.close, master_fd)
        resources.callback(os.close, slave_fd)
        device = os.ttyname(slave_fd)
        writer_fd = os.open(
            device,
            os.O_WRONLY | os.O_NOCTTY | os.O_NONBLOCK | os.O_CLOEXEC,
        )
        resources.callback(os.close, writer_fd)
        owner = ExclusiveSerialPort(device, lock_dir=tmp_path)
        owner.open()
        resources.callback(owner.close)
        payload = b"preexisting-writer-remains-live"

        # When: the already-open descriptor writes after the exclusive claim.
        written = os.write(writer_fd, payload)
        with selectors.DefaultSelector() as selector:
            selector.register(master_fd, selectors.EVENT_READ)
            ready = selector.select(timeout=1.0)

        # Then: the kernel accepts the write; TIOCEXCL does not evict existing writers.
        assert written == len(payload)
        assert ready
        assert os.read(master_fd, len(payload)) == payload
