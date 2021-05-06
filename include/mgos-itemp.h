#pragma once

#include <stdbool.h>
#include <stdint.h>

enum itemp_cmd {
  ITCMD_ADJUST = 0,
  ITCMD_COMFORT,
  ITCMD_SETBACK,
  ITCMD_WINDOW_OPEN,
  ITCMD_WINDOW_SHUT
};

struct itemp_status {
  bool busy, ok;
};

#ifdef __cplusplus
extern "C" {
#endif

bool mgos_itemp_send_cmd(uint32_t src, enum itemp_cmd cmd, int8_t arg);
bool mgos_itemp_setup_rf(bool reset);
bool mgos_itemp_status(struct itemp_status *st);

#ifdef __cplusplus
}
#endif
