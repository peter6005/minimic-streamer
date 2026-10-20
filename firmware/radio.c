// radio.c - see radio.h. Runs the transmit timing loop on core1: 3 slots per 6.144 ms frame at offsets
// 0/1536/3072 us on the hopping channels, each audio slot sent twice (the duplicates are required), plus
// an optional parity packet. Also handles pairing and the optional beacon monitor. See CODEC.md.
#include <string.h>
#include "radio.h"
#include "nrf24.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"

// ---- address + hop table (defaults belong together; see CODEC.md "Frequency hopping") ----------------
static uint8_t my_addr[4] = {0x12, 0x34, 0x56, 0x78};
static volatile bool addr_dirty = false;
static uint8_t HOP[16] = {24, 76, 18, 44, 66, 36, 16, 78, 68, 20, 62, 22, 38, 48, 42, 28};  // table for 12 34 56 78

// ---- frame timing ------------------------------------------------------------------------------------
#define SLOT_US   1536
#define FRAME_US  (4 * SLOT_US)
#define CE_HIGH_2 460          // CE-high for a 2-payload slot
#define CE_HIGH_1 300          // CE-high for a 1-payload slot
static const uint8_t LOWB[3] = {1, 0, 1};        // byte1 low bits for 0x18, 0x19, 0x1A
static const uint8_t TYPE[3] = {0x18, 0x19, 0x1A};
static const uint8_t FLAG[3] = {0x20, 0x20, 0x00};

// ---- frame ring (core0 fills, core1 drains) ----------------------------------------------------------
#define RING 256
static uint8_t ring[RING][84];
static volatile uint32_t head = 0, tail = 0;
static volatile uint32_t cnt_tx = 0, cnt_under = 0, cnt_drop = 0, cnt_rst = 0;

// ---- pairing (CODEC.md "Pairing") --------------------------------------------------------------------
static const uint8_t BEACON_ADDR[4] = {0xBC, 0xBC, 0xBC, 0xBC};
static const uint8_t PAIR_CH[4] = {10, 46, 28, 64};
#define PAIR_DWELL_US   765000
#define PAIR_WINDOW_US  30000000                 // pairing mode lasts 30 s; any number of receivers
static volatile int pair_req = 0;                // core0 -> core1: 1 = start, 2 = cancel
static volatile int pair_state = RADIO_PAIR_IDLE;
static volatile uint32_t pair_beacons = 0, pair_count = 0;
static volatile bool core1_ready = false, monitor = false;
static uint8_t paired_id[4];

// ---- beacon telemetry (pairing + monitor -> core0 status line) ---------------------------------------
static uint8_t last_beacon[32];
static volatile uint32_t beacon_seq = 0;
static volatile uint32_t beacon_us = 0, beacon_cnt = 0;

static void build_payload(uint8_t *out, int slot, uint8_t counter, const uint8_t *codec28) {
    out[0] = TYPE[slot];
    out[1] = (counter << 2) | LOWB[slot];
    if (codec28) memcpy(out + 2, codec28, 28); else memset(out + 2, 0, 28);
    out[30] = FLAG[slot];
    out[31] = 0;
}

static void burst_at(uint64_t at, uint32_t hi) {
    while (time_us_64() < at) tight_loop_contents();
    nrf_ce(1); busy_wait_us(hi); nrf_ce(0);
}

static bool in_list(uint8_t list[][4], int n, const uint8_t *id) {
    for (int i = 0; i < n; i++) if (!memcmp(list[i], id, 4)) return true;
    return false;
}

// reply to a receiver that asked to pair: send our address to the receiver's own address, ~6 ms after it beaconed
static void send_reply(const uint8_t *b, uint8_t ch) {
    uint8_t r[32] = {0};
    r[0] = (uint8_t)(((b[0] & 0xF0) + 0x20) | 0x10);     // new pairing session number
    r[1] = 0x35;
    // Reply bytes 4,5 depend on the receiver generation(?), which it announces in beacon byte 5:
    // 0x00 = N8 (reply 93 14), 0x06 = N6 (reply 92 08). See CODEC.md section 9.
    if (b[5] == 0x06) { r[4] = 0x92; r[5] = 0x08; } else { r[4] = 0x93; r[5] = 0x14; }
    memcpy(r + 6, b + 12, 10);                           // echo the receiver's own serial number
    memcpy(r + 16, my_addr, 4);                          // our address
    r[24] = 0x04; r[25] = 0x08; r[26] = 0x05;
    nrf_ce(0); nrf_cmd(NRF_FLUSH_TX); nrf_wreg1(NRF_REG_CONFIG, 0x7E);
    nrf_wreg(NRF_REG_TX_ADDR, b + 8, 4); nrf_wreg1(NRF_REG_RF_CH, ch);
    nrf_write_payload(r);
    busy_wait_us(5800);                                  // the receiver listens ~6 ms after its beacon
    nrf_ce(1); busy_wait_us(CE_HIGH_1); nrf_ce(0); busy_wait_us(300);
}

static void pair_run(void) {
    static uint8_t done[16][4], replied[16][4]; int ndone = 0, nreplied = 0;
    pair_state = RADIO_PAIR_SEARCH; pair_beacons = 0; pair_count = 0;
    uint64_t t_end = time_us_64() + PAIR_WINDOW_US; int ci = 0;
    while (time_us_64() < t_end && pair_req != 2) {
        uint8_t ch = PAIR_CH[ci]; ci = (ci + 1) & 3;
        nrf_setup_rx(ch, BEACON_ADDR);
        uint64_t dwell = time_us_64() + PAIR_DWELL_US;
        while (time_us_64() < dwell && time_us_64() < t_end && pair_req != 2) {
            if (!(nrf_cmd(NRF_NOP) & 0x40)) continue;
            uint8_t b[32]; nrf_read_payload(b); nrf_wreg1(NRF_REG_STATUS, 0x70);
            if (!((b[3] == 0x1E || b[3] == 0x16) && b[4] == 0x03)) continue;   // not a receiver beacon
            pair_beacons++; memcpy(last_beacon, b, 32); beacon_seq++;
            // Confirmation: a 0x16 beacon with the paired bit set, from a receiver we just replied to.
            // N8 also echo our address in bytes 22..25; N6 leaves them zero, so we key off the reply
            // list and the paired bit instead of the address. See CODEC.md section 9.
            bool confirm = b[3] == 0x16 && (b[29] & 0x08) &&
                           (in_list(replied, nreplied, b + 8) || memcmp(b + 22, my_addr, 4) == 0);
            if (confirm) {
                if (!in_list(done, ndone, b + 8)) {
                    if (ndone < 16) memcpy(done[ndone++], b + 8, 4);
                    memcpy(paired_id, b + 8, 4); pair_count++;
                }
            } else if ((b[28] & 0x80) && !(b[29] & 0x08) && !in_list(done, ndone, b + 8)) {
                send_reply(b, ch); nrf_setup_rx(ch, BEACON_ADDR);
                if (!in_list(replied, nreplied, b + 8) && nreplied < 16) memcpy(replied[nreplied++], b + 8, 4);
            }
        }
    }
    nrf_ce(0); nrf_cmd(NRF_FLUSH_RX);
    pair_state = pair_count ? RADIO_PAIR_DONE : RADIO_PAIR_TIMEOUT;
    pair_req = 0;
    nrf_setup_tx(my_addr);                               // back to transmit
}

static void __not_in_flash_func(core1_main)(void) {
    multicore_lockout_victim_init();                     // core0 pauses us briefly to read BOOTSEL
    nrf_setup_tx(my_addr);
    core1_ready = true;
    uint8_t counter = 0; uint64_t epoch = time_us_64() + 2000;
    uint8_t p18[32], p19[32], p1a[32], p39[32], p38[32];
    while (true) {
        if (pair_req == 1) { pair_run(); tail = head; epoch = time_us_64() + 2000; }
        if (addr_dirty) { nrf_wreg(NRF_REG_TX_ADDR, my_addr, 4); addr_dirty = false; }
        if (counter == 0) {                              // once per 48-frame cycle: has the radio browned out?
            if (nrf_rreg1(NRF_REG_CONFIG) != 0x7E || nrf_rreg1(NRF_REG_RF_SETUP) != 0x0B) {
                nrf_setup_tx(my_addr); cnt_rst++;        // re-initialise and count it (power/interference symptom)
            }
        }
        const uint8_t *fr = NULL;
        if (tail != head) { __dmb(); fr = ring[tail]; }
        else cnt_under++;
        build_payload(p18, 0, counter, fr ? fr + 0 : NULL);
        build_payload(p19, 1, counter, fr ? fr + 28 : NULL);
        build_payload(p1a, 2, counter, fr ? fr + 56 : NULL);
        if (fr) { uint32_t nt = tail + 1; tail = (nt == RING) ? 0 : nt; }
        memcpy(p39, p19, 32); p39[0] = 0x39;
        memcpy(p38, p18, 32); p38[0] = 0x38;
        uint8_t cA = HOP[(3 * counter) % 16], cB = HOP[(3 * counter + 1) % 16], cC = HOP[(3 * counter + 2) % 16];
        nrf_wreg1(NRF_REG_RF_CH, cA); nrf_cmd(NRF_FLUSH_TX); nrf_write_payload(p18); nrf_write_payload(p39); burst_at(epoch + 0 * SLOT_US, CE_HIGH_2);
        nrf_wreg1(NRF_REG_RF_CH, cB); nrf_cmd(NRF_FLUSH_TX); nrf_write_payload(p19); nrf_write_payload(p38); burst_at(epoch + 1 * SLOT_US, CE_HIGH_2);
        nrf_wreg1(NRF_REG_RF_CH, cC); nrf_cmd(NRF_FLUSH_TX); nrf_write_payload(p1a);                          burst_at(epoch + 2 * SLOT_US, CE_HIGH_1);
        if (monitor) {                                   // idle quarter: listen for receiver beacons
            static int mon_ch = 0, mon_frames = 0;
            if (++mon_frames >= 100) { mon_frames = 0; mon_ch = (mon_ch + 1) & 3; }
            nrf_setup_rx(PAIR_CH[mon_ch], BEACON_ADDR);
            while (time_us_64() < epoch + FRAME_US - 300) {
                if (!(nrf_cmd(NRF_NOP) & 0x40)) continue;
                uint8_t b[32]; nrf_read_payload(b); nrf_wreg1(NRF_REG_STATUS, 0x70);
                if ((b[3] == 0x1E || b[3] == 0x16) && b[4] == 0x03) {
                    beacon_us = (uint32_t)(time_us_64() - epoch); beacon_cnt = counter;
                    memcpy(last_beacon, b, 32); beacon_seq++;
                }
            }
            nrf_ce(0); nrf_cmd(NRF_FLUSH_RX); nrf_wreg1(NRF_REG_CONFIG, 0x7E); nrf_wreg1(NRF_REG_STATUS, 0x70);
        }
        counter = (counter + 1) % 48; cnt_tx++;
        epoch += FRAME_US;
        if ((int64_t)(time_us_64() - epoch) > FRAME_US) epoch = time_us_64() + 2000;
    }
}

// ---- public API --------------------------------------------------------------------------------------
void radio_launch(void) {
    nrf_bus_init();
    multicore_launch_core1(core1_main);
}
bool radio_core_ready(void) { return core1_ready; }

bool radio_push_frame(const uint8_t frame84[84]) {
    uint32_t nh = head + 1; if (nh == RING) nh = 0;
    if (nh == tail) { cnt_drop++; return false; }        // full: drop
    memcpy(ring[head], frame84, 84); __dmb(); head = nh;
    return true;
}
uint32_t radio_depth(void) { return (head + RING - tail) % RING; }

void radio_set_addr(const uint8_t addr[4]) { memcpy(my_addr, addr, 4); addr_dirty = true; }
void radio_set_hop(const uint8_t hop[16]) { memcpy(HOP, hop, 16); }
void radio_set_monitor(bool on) { monitor = on; }
void radio_start_pairing(void) { pair_req = 1; }
void radio_button_pairing(void) {
    pair_req = (pair_state == RADIO_PAIR_SEARCH || pair_state == RADIO_PAIR_REPLIED) ? 2 : 1;
}

void radio_get_addr(uint8_t out[4]) { memcpy(out, my_addr, 4); }
int radio_pair_state(void) { return pair_state; }
bool radio_pairing_active(void) { return pair_req == 1 || pair_state == RADIO_PAIR_SEARCH || pair_state == RADIO_PAIR_REPLIED; }
uint32_t radio_pair_beacons(void) { return pair_beacons; }
uint32_t radio_pair_count(void) { return pair_count; }
void radio_get_paired_id(uint8_t out[4]) { memcpy(out, paired_id, 4); }
uint32_t radio_tx_count(void) { return cnt_tx; }
uint32_t radio_under_count(void) { return cnt_under; }
uint32_t radio_drop_count(void) { return cnt_drop; }
uint32_t radio_reset_count(void) { return cnt_rst; }

bool radio_take_beacon(uint32_t *seq, uint8_t buf[32], uint32_t *us, uint32_t *counter) {
    if (beacon_seq == *seq) return false;
    *seq = beacon_seq; memcpy(buf, last_beacon, 32);
    *us = beacon_us; *counter = beacon_cnt;
    return true;
}
