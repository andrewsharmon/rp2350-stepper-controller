#pragma once

// I2C target on the Qwiic port (boards that define BOARD_I2C_SDA_GPIO).
// Carries the binary protocol's requests (protocol.h) without framing:
//
//   write  [type][payload...]       one request per write transaction
//   read   [n][type][payload...]    the reply to the last request; n = 1 + payload
//                                   length, or 0 while it is still being handled
//                                   (read again). Telemetry is not pushed over
//                                   I2C: poll with STATUS_REQ (~50-100 Hz).
//
// The interrupt only collects bytes; requests run in the core 0 loop
// (i2c_target_poll), since some write flash or print.

#include <stdbool.h>

// False on boards without I2C target pins.
bool i2c_target_init(void);
void i2c_target_poll(void);
