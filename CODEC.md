# The MiniMic radio link and audio codec

This document describes the over-the-air format of a 2.4 GHz wireless microphone built on the
Nordic nRF24L01+ radio, and the audio codec it uses: the radio framing, the transform, and the bit
stream. It is enough to write both an encoder (this project's firmware has one) and a decoder.

**Is this a known codec?** No standard codec matches the bit stream, but the *design* is from the
CELT / Opus family: an MDCT front end with **gain-shape coding**, where each frequency band's energy
(a log size) is sent separately from its shape, and the shape is a set of signed integer **pulses**
(like CELT's pyramid vector quantiser, where the pulse count sets the band's loudness). Commercial and
hobbyist nRF24L01+ audio links instead send simple PCM or A-law, not a transform codec. The difference
from Opus/CELT is the entropy coding: this codec packs fixed-width fields (pulses as
`[run][1][sign][b]`, a 9-bit size, zero padding) into 16-bit words read LSB-first, rather than using a
range coder — so it is a **proprietary codec in the CELT family**, not something an existing library
will decode. (Reference: the CELT paper, "Constrained-Energy Lapped Transform", arXiv:1602.05311.)

Contents

1. [Radio link](#1-radio-link)
2. [Packet payload](#2-packet-payload)
3. [Audio transform](#3-audio-transform)
4. [Block bit stream](#4-block-bit-stream)
5. [Decoding a block](#5-decoding-a-block)
6. [Rules an encoder must follow](#6-rules-an-encoder-must-follow)
7. [How the encoder in this project works](#7-how-the-encoder-in-this-project-works)
8. [Typical streams and open points](#8-typical-streams-and-open-points)
9. [Pairing](#9-pairing)

---

## 1. Radio link

| Setting | Value |
|---|---|
| Radio | nRF24L01+ (Enhanced ShockBurst framing, auto-ack off) |
| Data rate | 2 Mbps |
| Address | 4 bytes, `12 34 56 78` (in the order written to TX_ADDR). Each microphone has its own address. A receiver learns it when pairing (section 9). |
| CRC | 2 bytes |
| Payload | fixed 32 bytes |
| Output power | -12 dBm (RF_SETUP = `0x0B`) |
| CONFIG | `0x7E` (power up, TX, 2-byte CRC, all IRQs masked) |

### Frame timing

The link runs in **frames of 6.144 ms** (162.76 frames per second). Each frame carries two audio
blocks of 3.072 ms. A frame has three transmit slots, followed by an idle quarter:

| Slot | Start in frame | Packets sent (in this order) |
|---|---|---|
| 0 | 0 µs | `0x18` (block A), then `0x39` (copy of block B) |
| 1 | 1536 µs | `0x19` (block B), then `0x38` (copy of block A) |
| 2 | 3072 µs | `0x1A` (parity) |
| — | 4608 µs | idle |

Every block goes out twice, in different slots and on different channels. The copies are
**required**: a receiver does not play a stream without them. The `0x1A` parity packet is optional.

### Frequency hopping

Every slot uses its own RF channel (RF_CH value, i.e. 2400 + n MHz). With the frame counter
`c` (0..47, see below):

```
HOP = [24, 76, 18, 44, 66, 36, 16, 78, 68, 20, 62, 22, 38, 48, 42, 28]
channel(slot) = HOP[(3*c + slot) mod 16]        slot = 0, 1, 2
```

The receiver follows the hop pattern using the counter inside each packet, so the counter in byte 1
must match the channel used. The table above belongs to address `12 34 56 78`. The receiver derives
the pattern from the transmitter's address: after pairing with a different address it no longer
follows this table. A sender that loops a recording must loop a whole number of
48-frame cycles.

---

## 2. Packet payload

All five packet types have the same 32-byte layout:

| Byte | Content |
|---|---|
| 0 | Type: `0x18` block A, `0x19` block B, `0x1A` parity. The copies use `0x38` (copy of `0x18`) and `0x39` (copy of `0x19`), i.e. type with bit `0x20` set. All other bytes of a copy are identical to the original. |
| 1 | `(c << 2) \| low`, frame counter `c` = 0..47 (wraps); `low` = 1 for `0x18`, 0 for `0x19`, 1 for `0x1A` |
| 2..29 | 28 bytes of codec data (one block, see section 4) |
| 30 | Input flag: `0x20` = line/jack input, `0x10` = built-in microphone; `0x00` in the `0x1A` packet |
| 31 | `0x00` |

The `0x1A` payload bytes 2..29 are the XOR of the `0x18` and `0x19` bytes 2..29.

There is no checksum above the radio's own CRC. A receiver plays whatever valid packets arrive.
A block of 28 zero bytes is clean silence.

---

## 3. Audio transform

| Item | Value |
|---|---|
| Sample rate | `FS = 64 / 3.072 ms = 20833.33 Hz` |
| Transform | MDCT, N = 64 coefficients, window length 128, hop 64 samples (3.072 ms) |
| Window | sine: `w[m] = sin(π (m + 0.5) / 128)`, m = 0..127 |
| Bin spacing | 162.76 Hz; bin k is centred at `(k + 0.5) · 162.76 Hz` |
| Coded bandwidth | in practice up to about bin 47..55 (~7.5..9 kHz) |

Coefficient k of block n (block n uses input samples `64n .. 64n+127`):

```
X[k] = Σ_{m=0..127}  x[64n + m] · w[m] · cos( π/64 · (m + 0.5 + 32) · (k + 0.5) )
```

There is no extra scale factor; the level constant in section 4 assumes this form, with input
samples normalized to ±1.0. Block A (`0x18`) and block B (`0x19`) of a frame are two consecutive
blocks, A first. A decoder uses the matching IMDCT with the same sine window and overlap-adds
consecutive blocks.

The original microphone has an analog high-pass of about 45 Hz in front of the converter. That is
part of its sound, not of the codec.

---

## 4. Block bit stream

### Bit order

The 28 codec bytes form 14 words of 16 bits, **big-endian** byte pairs:

```
W[i] = (byte[2i] << 8) | byte[2i+1]          i = 0..13
```

The block is **one continuous bit stream of 224 bits**. Words are taken in order, and each word is
read **least-significant bit first**:

```
bit[j] = (W[j / 16] >> (j % 16)) & 1         j = 0..223
```

Values that span several bits (sizes below) are also stored LSB first. Codes cross word boundaries
freely.

### Pulses

The stream is a run-length coded list of spectral **pulses**, from the lowest bin upward:

```
pulse = [r zero bits] 1 a b [value]

    r      number of zero bits before the 1 (run length) -> skips r bins
    a      sign: 0 = positive, 1 = negative coefficient
    b = 1  FINE pulse:   value = 9-bit size S (0..511)
    b = 0  COARSE pulse: value = 3-bit code v (0..7), meaning size S = 32 · (v + 1)
```

A position counter `p` (the current bin) starts at 0:

```
bin = p + r                       the pulse belongs to this bin
after a FINE pulse:    p = bin + 1
after a COARSE pulse:  p = bin    (the next run counts from this same bin, so the next run is >= 1)
```

So the first pulse of a block with `r = 0` is bin 0. The fine pulses "use up" their bin, and the coarse pulses don't.
Both forms end on the bin they describe; this rule is what puts every following pulse on the right
bin.

### Size scale

Sizes are logarithmic, **32 steps per doubling** (one step = 0.188 dB):

```
|X| = 2 ^ ((S - 177) / 32)
```

Fine and coarse pulses use the same scale. A coarse pulse is exactly a fine pulse whose size is a
multiple of 32 (v = 0..7 ↔ S = 32, 64, ..., 256), at the cost of 6 bits instead of 12. With the
normalization of section 3, a full-scale sine reaches sizes of about 330..347. The largest
possible size, 511, is about 31 dB above that.

### End of the block

After the last pulse the rest of the block is zero bits. Their number counts as a final run:

```
end position  =  p (after the last pulse)  +  number of trailing zero bits
```

The receiver behaves as if it keeps reading pulses until the position has reached about bin **52**. The original
microphone always ends its blocks at an end position between 52 and 57. If a block's bits run out
earlier, the receiver goes on reading past the end of the block. That decodes garbage, which plays
as sharp crackle. See section 6.

---

## 5. Decoding a block

```
def decode_block(data28):
    W = [(data28[2*i] << 8) | data28[2*i+1] for i in range(14)]
    bits = [(W[j // 16] >> (j % 16)) & 1 for j in range(224)]
    X = [0.0] * 64
    i, p, r = 0, 0, 0
    while i < 224:
        if bits[i] == 0:
            r += 1; i += 1; continue
        if i + 3 > 224: break
        a, b = bits[i + 1], bits[i + 2]; i += 3
        n = 9 if b else 3
        if i + n > 224: break
        val = sum(bits[i + q] << q for q in range(n)); i += n
        S = val if b else 32 * (val + 1)
        k = p + r
        if k < 64:
            X[k] = (-1 if a else 1) * 2 ** ((S - 177) / 32)
        p = k + 1 if b else k
        r = 0
    return X            # then IMDCT (sine window) and overlap-add with the previous block
```

Bins without a pulse are zero.

---

## 6. Rules an encoder must follow

1. **Positions strictly increase.** Never put two pulses on one bin. After a coarse pulse the next run
   must be at least 1.
2. **Reach the end position.** For every block:
   `trailing zero bits >= 53 - p_after_last_pulse` (and keep at least a few trailing zeros).
   Leaving the end position below ~52 makes the receiver read past the block (crackle).
   Pulses cost bits but move the position forward, so the bit budget and this rule must be checked
   together. Adding a pulse can break the rule even when there are bits to spare.
3. **Use both duplicates and a correct counter/hop channel** (sections 1–2).
4. **Avoid a coarse pulse with v = 0 that is directly followed by another pulse (run 0).** The original
   microphone very rarely produces this pattern, and its meaning is not known (it is not a normal
   pulse).
5. An all-zero block is always safe (silence).

Signs matter: the sign bit is the real sign of the MDCT coefficient, for coarse and fine
pulses alike. Correct signs are needed for clean overlap-add between blocks.

---

## 7. How the encoder in this project works

`codec/encoder.c` (portable C; also builds on a PC, see `codec/pc_encode.c`):

1. **Resample** 48 kHz mono input to 20833.33 Hz with linear interpolation. The ratio is exactly
   288/125, so there is no long-term drift.
2. **MDCT** every 64 new samples (section 3) and take sizes `L[k] = 32·log2|X[k]| + 177`.
3. **Bins 0..3** (0..650 Hz) are always written, in order, with run 0:
   - coarse if the bin is more than 96 steps (18 dB) below the block's loudest bin, or if its size is
     within 4 steps of a multiple of 32 (then coarse is nearly exact);
   - fine otherwise.
4. **Bins 4..46** (650 Hz .. 7.6 kHz), candidates are bins with size >= 13:
   - up to 4 of the loudest bins within 32 steps (6 dB) of the loudest one become **fine** pulses;
   - then the strongest bin of every group of 5 bins (if within ~60 dB of the loudest) becomes a
     **coarse** pulse, so weak regions are still present;
   - then the remaining bins, loudest first, down to ~40 dB below the loudest, as **coarse** pulses;
   - each pulse is only added if the block still fits **and** keeps the end-position rule
     (section 6, rule 2, with target 53 and at least 6 trailing zeros).
5. **Write** the pulses in bin order with the position rule of section 4. Coarse code
   `v = round(L / 32) - 1` (clamped 0..7), fine size `round(L)` (clamped 0..511), sign from the
   coefficient.
6. Two consecutive blocks form a frame (`0x18`, `0x19`). The parity block is their XOR.

The encoder needs about 9 ms of audio (the 128-sample window plus the pairing into frames). On an
RP2350 it uses only a small part of one core.

---

## 8. Typical streams and open points

What the original microphone typically sends (speech/music, line input):

- bins 0..3 almost always present, each as one pulse with run 0, fine or coarse;
- about 15–25 pulses per block, of which roughly 3–9 are fine, coded up to bin ~40–55;
- weak coarse pulses (v = 0..2) also in quiet regions, the microphone's own noise;
- fine sizes that are almost never close to a multiple of 32, because a coarse pulse codes those
  exactly;
- end position 52..57, typically 9–16 trailing zero bits.

Not fully known:

- how the receiver derives the hop table from the transmitter address (only the table for
  `12 34 56 78` is known);
- the meaning of the rare "coarse v = 0, then run 0" pattern (section 6, rule 4);
- what the receiver does with bins that have no pulse (they behave as zero, but there may be a
  noise floor or comfort-noise detail);
- why the microphone sometimes codes a bin as fine when coarse would be close enough (it seems to depend on
  its own internal noise, so it cannot be reproduced from a clean signal; it does not matter for
  sound quality).

---

## 9. Pairing

A receiver remembers one transmitter address. Pairing teaches it a new one. Radio settings are as in
section 1 (2 Mbps, 4-byte addresses, 2-byte CRC, auto-ack off, 32-byte payloads).

### Receiver beacons

Receivers regularly send **beacons** to the shared address `BC BC BC BC`, on the four channels
**10, 46, 28 and 64**, changing channel every few hundred milliseconds. A transmitter that wants to
pair listens there: CONFIG `0x3F` (receive), RX_ADDR_P0 = `BC BC BC BC`, RX_PW_P0 = 32, only pipe 0
enabled. It stays about 0.77 s on each of the four channels in turn.

Beacon payload (32 bytes):

| Byte | Content |
|---|---|
| 0 | Pairing session number of the last pairing (`0x10` from the factory, then the value from the transmitter's reply) |
| 1 | Low 2 bits: receiver-specific (seen `01`, `03`). After pairing the upper bits come from reply byte 1 (`0x35` → `0x35`/`0x37`) |
| 2 | RF channel this beacon is sent on |
| 3 | `0x1E` normal beacon, `0x16` confirmation right after a pairing |
| 4, 5 | `03 00` on N8 receivers; `03 06` on N6 receivers. Byte 5 marks the generation(?) and selects the reply's bytes 4, 5 (below) |
| 6 | `00`, after pairing the value of reply byte 4 (`0x93`) |
| 7 | `0x1A` normal, `0x12` in a confirmation |
| 8..11 | Receiver ID, which is also the receiver's own radio address |
| 12..21 | Receiver serial number, ASCII |
| 22..25 | In a confirmation: the transmitter address the receiver has accepted. Otherwise `00` |
| 26, 27 | `7E 00` |
| 28 | Bit `0x80`: pairing window open (set for a while after the receiver is switched on) |
| 29 | `0x43`; bit `0x08` set (`0x4B`) once the receiver has been paired in this session |
| 30, 31 | `00 00` |

A receiver **asks to be paired** when byte 28 has bit `0x80` set and byte 29 has bit `0x08` clear.
Restarting a receiver opens this window.

### Transmitter reply

To a receiver that asks, the transmitter sends **one packet** to the receiver's own address (beacon
bytes 8..11), on the channel the beacon came on, with CONFIG `0x7E` (transmit):

| Byte | Content |
|---|---|
| 0 | New pairing session number. The original microphone counts `0x30`, `0x50`, `0x70`, ... It must differ from beacon byte 0. |
| 1 | `0x35` |
| 2, 3 | `00 00` |
| 4, 5 | `93 14` for a receiver whose beacon byte 5 is `00` (N8). `92 08` when beacon byte 5 is `06` (N6). Wrong values here are silently rejected and pairing never confirms |
| 6..15 | The receiver's serial number (copied from beacon bytes 12..21) |
| 16..19 | The transmitter's address (as written to TX_ADDR) |
| 20..23 | `00` |
| 24..26 | `04 08 05` |
| 27..31 | `00` |

**Timing matters:** the reply must go out about **5.9 ms after the beacon**. A reply sent at once
arrives before the receiver listens, and is ignored. There is no acknowledgement. If the reply is
lost, the receiver keeps asking, and the transmitter simply replies to the next request.

### Confirmation

The receiver then sends confirmation beacons (byte 3 = `0x16`) with byte 29's `0x08` bit set (paired).
Its normal beacons from then on carry the new session number in byte 0 and the paired bit in byte 29.
The transmitter returns to its normal audio stream (section 1), and the receiver plays it.

How the receiver marks *which* transmitter it accepted differs by generation:

- **N8** (beacon byte 5 = `00`) echo the accepted transmitter address in bytes 22..25.
- **N6** (beacon byte 5 = `06`) leave bytes 22..25 as `00` and only set the paired bit.

So a transmitter should treat a `0x16` beacon with the paired bit set, from a receiver it has just
replied to, as the confirmation — rather than matching on the echoed address, which only the newer
receivers provide.

