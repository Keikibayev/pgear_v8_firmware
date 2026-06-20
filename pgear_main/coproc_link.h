// coproc_link.h — UART link to the ADS1256 coproc.  [Phase 3]
//   - RX byte-state-machine parser (dispatch on start1): SensorPacketV2 (0xAA55)
//     + CalStatePacket (0xAA77); CRC check; link-age tracking; resync counters.
//   - TX: DownstreamPacket (0xBB66, ~10 Hz, estop+fault flags + tareSeq) and
//     CalAdjustPacket (0xBB55, on demand).
//   - Converts calibrated force N -> joint torque (moment arm) into the snapshot.
#pragma once
#include "protocol.h"
// TODO P3: coproc_link_init(); poll(); latest force/torque; send downstream/caladjust.
