#include <stdint.h>
/* ESP32's endian.h fails without stdint.h prior. */
#include <endian.h>
#include <stddef.h>

#include <mgos-cc1101.h>
#include <mgos-helpers/log.h>
#include <mgos-itemp.h>

enum itctl { ITCTL_RC = 16, ITCTL_WS = 32 };
enum itnop { ITNOP = 8 };
enum itsig {
  ITSIG_RC_ADJUST = 0,
  ITSIG_RC_COMFORT = 2,
  ITSIG_RC_SETBACK = 3,
  ITSIG_WS_OPEN = 1,
  ITSIG_WS_SHUT = 0
};
enum itsyn { ITSYN = 126 };

struct itmsg {
  union {
    struct {
      uint8_t syn, seq, ctl, src[3];
#if BYTE_ORDER == BIG_ENDIAN
      uint8_t nop : 5;
      uint8_t sig : 3;
#else
      uint8_t sig : 3;
      uint8_t nop : 5;
#endif
      int8_t arg;
      union {
        uint8_t crc8[2];
        uint16_t crc;
      };
    };
    uint8_t val[10];
  };
};

/* CRC16-CCITT, custom final XOR */
static uint16_t cmd_crc16(uint8_t *data, size_t sz) {
  uint16_t crc = 0x1d0f, i;
  while (sz--) {
    crc ^= *data++ << 8;
    for (i = 8; i--;) crc = crc << 1 ^ (crc & 0x8000 ? 0x1021 : 0);
  }
  return crc ^ 0x1643;
}

/* Reverse bit order. */
static void cmd_stib(uint8_t *bits) {
  static const uint8_t map[] = {0, 8, 4, 12, 2, 10, 6, 14,
                                1, 9, 5, 13, 3, 11, 7, 15};
  *bits = map[*bits >> 4] | map[*bits & 15] << 4;
}

/* Stuff a 0 bit after every streak of five 1s, except in the syn byte, a la
 * SDLC/HDLC.  Write up to ITMSG_STUFFED_MAXSZ bytes to `buf'.  Return the
 * resulting size in bits. */
#define ITMSG_RF_BITS_MAX (8 + (sizeof(struct itmsg) - 1) * 8 / 5 * 6)
#define ITMSG_RF_BUFSZ ((ITMSG_RF_BITS_MAX + 7) / 8)
static size_t cmd_bitstuff(const struct itmsg *msg, uint8_t *buf) {
  const uint8_t *from = msg->val;
  uint8_t *to = buf;
  uint32_t acc = *from++;
  int acc_mid = 0, acc_sz = 8;  // The syn byte can and even has to have six 1s.

  for (;;) {
    while (acc_sz - 8 >= acc_mid) *to++ = acc >> (acc_sz -= 8);  // Ready data.
    if (acc_mid < 5) {  // Need more to may warrant stuffing?
      if (from == msg->val + sizeof(*msg)) break;  // End of msg.
      acc = acc << 8 | *from++;                    // Source another byte.
      acc_mid += 8;
      acc_sz += 8;
    }
    while (acc_mid >= 5)
      if ((acc >> (acc_mid - 5) & 31) != 31)  // Five 1s in a row?
        acc_mid--;                            // No.
      else {                                  // Stuff a 0.
        acc_mid -= 5;
        acc_sz++;
        acc = (acc >> acc_mid << (acc_mid + 1)) | (acc & ((1 << acc_mid) - 1));
      }
  }

  if (acc_sz >= 8) *to++ = acc >> (acc_sz -= 8);
  if (acc_sz) *to = acc << (8 - acc_sz);
  return (to - buf) * 8 + acc_sz;
}

static const char *cmd_str(enum itemp_cmd cmd) {
  switch (cmd) {  // clang-format off
    case ITCMD_ADJUST: return "ADJUST";
    case ITCMD_COMFORT: return "COMFORT";
    case ITCMD_SETBACK: return "SETBACK";
    case ITCMD_WINDOW_OPEN: return "WINDOW_OPEN";
    case ITCMD_WINDOW_SHUT: return "WINDOW_SHUT";
    default: return "<invalid>";
  };  // clang-format on
}

#define ITMSG_COPIES (300 - 1)  // Repeat enough: the receiver isn't always on.
bool mgos_itemp_send_cmd(uint32_t src, enum itemp_cmd cmd, int8_t arg,
                         uint32_t quiet_us, mgos_cb_t cb, void *opaque) {
  /* Code up the command signal. */
  enum itctl ctl;
  enum itsig sig;
  switch (cmd) {  // clang-format off
    case ITCMD_ADJUST: ctl = ITCTL_RC; sig = ITSIG_RC_ADJUST; break;
    case ITCMD_COMFORT: ctl = ITCTL_RC; sig = ITSIG_RC_COMFORT; break;
    case ITCMD_SETBACK: ctl = ITCTL_RC; sig = ITSIG_RC_SETBACK; break;
    case ITCMD_WINDOW_OPEN: ctl = ITCTL_WS; sig = ITSIG_WS_OPEN; break;
    case ITCMD_WINDOW_SHUT: ctl = ITCTL_WS; sig = ITSIG_WS_SHUT; break;
    default: FNERR_RETF("cmd %d unknown", cmd);
  };  // clang-format on

  /* Build the message. */
  static uint8_t itmsg_seq = 0;
  struct itmsg m = {
    syn : ITSYN,
    seq : itmsg_seq++,
    ctl : ctl,
    src : {src >> 16, src >> 8, src},
    nop : ITNOP,
    sig : sig,
    arg : arg
  };
  uint8_t *bits = &m.seq;  // Revert bit order to LSB first, a la RS-232.
  while (bits < m.crc8) cmd_stib(bits++);  // Skip the syn byte, it's symmetric.
  m.crc = htons(cmd_crc16(&m.seq, m.crc8 - &m.seq));  // Omit the syn byte too.

  /* See cmd_bitstuff() above: one extra bit possible per source five. */
  uint8_t *rf = malloc(ITMSG_RF_BUFSZ);
  if (!rf) FNERR_RETF("%s(%u): failed", "malloc", ITMSG_RF_BUFSZ);
  size_t sz = cmd_bitstuff(&m, rf);  // Bit stuffing.

  if (mgos_sys_config_get_itemp_debug())
    FNLOG(LL_INFO, "%06X(%u us) → %s(%d): rf(%zu) %08x%08x%04x%02x%02x", src,
          quiet_us, cmd_str(cmd), arg, sz,
          (rf[0] << 24) | (rf[1] << 16) | (rf[2] << 8) | rf[3],
          (rf[4] << 24) | (rf[5] << 16) | (rf[6] << 8) | rf[7],
          (rf[8] << 8) | rf[9], rf[10] & 255 << (sz > 88 ? 0 : 88 - sz),
          rf[11] & 255 << (sz > 96 ? 0 : 96 - sz));

  struct mgos_cc1101_tx_req req = {
    data : rf,
    len : sz,
    copies : ITMSG_COPIES,
    free_data : true,
    quiet_us : quiet_us,
    cb : cb,
    opaque : opaque
  };
  bool ok = false;
  ok = TRY_GT(mgos_cc1101_tx, mgos_cc1101_get_global_locked(), &req);
err:
  if (!ok) free(rf);
  mgos_cc1101_put_global_locked();
  return ok;
}

static bool mgos_itemp_setup_rf_cb(struct mgos_cc1101 *cc1101,
                                   struct mgos_cc1101_regs *regs,
                                   void *opaque) {
  TRY_RETF(mgos_cc1101_set_data_rate, cc1101, regs, 9.6);
  TRY_RETF(mgos_cc1101_set_frequency, cc1101, regs, 868.35);
  TRY_RETF(mgos_cc1101_set_deviation, cc1101, regs, 25);
  TRY_RETF(mgos_cc1101_set_modulation, cc1101, regs, CC1101_MOD_FORMAT_GFSK, 0,
           true, false);  // GFSK+Manchester, no preamble, no FEC/interleaving.
  CC1101_REGS_REG(regs, PKTCTRL0).WHITE_DATA = false;
  CC1101_REGS_REG(regs, PKTCTRL0).CRC_EN = false;
  return true;
}

bool mgos_itemp_setup_rf(bool reset) {
  bool ok = false;
  struct mgos_cc1101 *cc1101 = mgos_cc1101_get_global_locked();
  if (!cc1101) return false;
  if (reset) TRY_GT(mgos_cc1101_reset, cc1101);
  TRY_GT(mgos_cc1101_write_reg, cc1101, CC1101_PATABLE, 192);
  ok = TRY_GT(mgos_cc1101_mod_regs, cc1101, CC1101_PKTCTRL0, CC1101_DEVIATN,
              mgos_itemp_setup_rf_cb, NULL);
err:
  mgos_cc1101_put_global_locked();
  return ok;
}

bool mgos_itemp_init() {
  if (mgos_sys_config_get_itemp_auto_setup()) mgos_itemp_setup_rf(true);
  return true;
}
