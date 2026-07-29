# Shared UDP v1 Arduino Library

This is the portable Arduino/host implementation of the bridge's authenticated
UDP v1 envelope. It uses explicit network-order serialization; it never casts a
packet into a C struct.

- Maximum payload: 256 bytes; maximum datagram: 306 bytes.
- Header: `EDU1`, version, message type, payload length, 8-byte sender ID,
  nonzero boot epoch, sequence, and sender monotonic milliseconds.
- CRC16-CCITT-FALSE covers header and payload.
- The first 16 bytes of HMAC-SHA256 cover header, payload, and CRC. Keys must
  contain at least 32 bytes.
- The receiver accepts forward modulo sequence deltas `1..1024`, retires the
  previous 32 epochs, and uses local receipt time for the 750 ms stale gate.

The fixed bridge vector is tested in `tests/protocol_test.cpp`. The library has
no runtime dependency beyond the C++ standard library and is suitable for
Arduino CLI `--libraries` use.
