# Vehicle/HMI UDP v1

All integer fields use network byte order. Datagrams are unicast and are bounded
to 306 bytes. No field is decoded past the authenticated boundary.

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 4 | Magic `EDU1` |
| 4 | 1 | Version `1` |
| 5 | 1 | Message type: telemetry `1`, selection `2`, ACK `3`, status `4` |
| 6 | 2 | Payload length, maximum 256 |
| 8 | 8 | ASCII sender ID, right-NUL-padded |
| 16 | 8 | Random nonzero boot epoch |
| 24 | 4 | Modulo-`uint32` sequence |
| 28 | 4 | Sender monotonic milliseconds, informational only |
| 32 | N | Explicit typed payload |
| 32+N | 2 | CRC16-CCITT-FALSE over header and payload |
| 34+N | 16 | First 16 bytes of HMAC-SHA256 over header, payload, and CRC |

The receiver binds each sender ID and current boot epoch to its provisioned
source IP/port. It accepts only forward modulo sequence deltas `1..1024`,
retires the previous 32 boot epochs, and uses local steady receipt time for the
0.75-second telemetry freshness gate. Sender time never establishes freshness.

Strings in payloads are UTF-8 with a one-byte length prefix and contract bounds.
Telemetry fixed fields are `>HBBffBB`; selection fields are `>HQQB`; ACK fields
are `>HQQBBB`; mission-status fields are `>HIBBB`. Strings follow those fixed
fields in Todo 1 contract order.

Golden vector key: bytes `00` through `1f`; type `1`; sender `CAR-01`; epoch
`0102030405060708`; sequence `fffffffe`; source millis `10203040`; payload
`010203`; CRC `4450`:

```text
45445531010100034341522d303100000102030405060708fffffffe102030400102034450affe1d99aa17475115930f8a10f67f52
```

The committed example configuration is intentionally unusable. Deployment must
provide numeric reserved peer addresses, owned UDP ports, sender IDs, and a
local file containing at least 32 random key bytes encoded as hexadecimal.
