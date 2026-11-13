"""
can_golden.py - bit-exact reference model of the CAN 2.0A/B data-link layer.

  * crc15(bits)                      - CRC-15 (poly 0x4599) over destuffed bits
  * encode_frame(...)                - full on-the-wire bit sequence (stuffed) of a frame,
                                       SOF .. EOF, with ACK slot driven dominant by a receiver
  * decode_stream(bits)              - independent protocol decoder ("logic analyzer"):
                                       finds SOFs, destuffs, parses std/ext frames, checks
                                       stuff rule, CRC, delimiters, ACK, EOF; reports errors
Written independently of the RTL so it can be used as a golden model.
"""

CRC_POLY = 0x4599


def crc15(bits):
    crc = 0
    for b in bits:
        fb = b ^ ((crc >> 14) & 1)
        crc = (crc << 1) & 0x7FFF
        if fb:
            crc ^= CRC_POLY
    return crc


def _bits(value, n):
    return [(value >> (n - 1 - i)) & 1 for i in range(n)]


def unstuffed_header_data(can_id, ide, rtr, dlc, data):
    """SOF .. last data bit (destuffed)."""
    b = [0]                                       # SOF
    if not ide:
        b += _bits(can_id & 0x7FF, 11) + [rtr, 0, 0]          # ID, RTR, IDE, r0
    else:
        b += _bits((can_id >> 18) & 0x7FF, 11) + [1, 1]       # base ID, SRR, IDE
        b += _bits(can_id & 0x3FFFF, 18) + [rtr, 0, 0]        # ext ID, RTR, r1, r0
    b += _bits(dlc, 4)
    nbytes = 0 if rtr else min(dlc, 8)
    for i in range(nbytes):
        b += _bits(data[i], 8)
    return b


def stuff(bits):
    out, run, last = [], 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)       # stuff bit
            last, run = 1 - b, 1
    return out


def encode_frame(can_id, ide=0, rtr=0, dlc=0, data=b"", acked=True):
    hd = unstuffed_header_data(can_id, ide, rtr, dlc, list(data) + [0] * 8)
    crc = crc15(hd)
    stuffed = stuff(hd + _bits(crc, 15))
    tail = [1, 0 if acked else 1, 1] + [1] * 7     # CRC delim, ACK slot, ACK delim, EOF
    return stuffed + tail, crc


class Frame:
    def __init__(self):
        self.id = 0; self.ide = 0; self.rtr = 0; self.dlc = 0; self.data = []
        self.crc_rx = None; self.crc_calc = None; self.ok = False; self.error = None
        self.start = 0; self.end = 0; self.acked = None; self.nstuff = 0

    def __repr__(self):
        kind = "EXT" if self.ide else "STD"
        if self.error:
            return f"[bit {self.start:6d}] ERROR-FRAME  ({self.error})"
        d = " ".join(f"{x:02X}" for x in self.data)
        return (f"[bit {self.start:6d}] {kind} ID=0x{self.id:0{8 if self.ide else 3}X} "
                f"{'RTR ' if self.rtr else ''}DLC={self.dlc:2d} DATA=[{d}] "
                f"CRC=0x{self.crc_rx:04X} stuffbits={self.nstuff} ACK={'yes' if self.acked else 'NO'}")


def decode_stream(bits, idle_needed=10):
    """Decode a list of sampled bus bits. Returns list of Frame objects."""
    frames, i, ones = [], 0, 0
    n = len(bits)
    while i < n:
        if bits[i] == 1:
            ones += 1; i += 1; continue
        if ones < idle_needed:              # dominant but not after bus idle -> skip
            ones = 0; i += 1; continue
        # ---- SOF found
        f = Frame(); f.start = i
        destuffed, run, last, j = [], 0, None, i
        stuff_err = False

        def take():
            nonlocal j, run, last, stuff_err
            while True:
                if j >= n:
                    raise IndexError
                b = bits[j]; j += 1
                if run == 5:                # this is a stuff bit
                    if b == last:
                        stuff_err = True
                        raise ValueError("stuff error")
                    f.nstuff += 1
                    run, last = 1, b
                    continue
                run = run + 1 if b == last else 1
                last = b
                destuffed.append(b)
                return b

        try:
            take()                                           # SOF
            base = sum(take() << (10 - k) for k in range(11))
            srr_rtr = take(); ide = take()
            if ide:
                ext = sum(take() << (17 - k) for k in range(18))
                f.id = (base << 18) | ext; f.rtr = take(); take(); take()
            else:
                f.id = base; f.rtr = srr_rtr; take()
            f.ide = ide
            f.dlc = sum(take() << (3 - k) for k in range(4))
            nbytes = 0 if f.rtr else min(f.dlc, 8)
            for _ in range(nbytes):
                f.data.append(sum(take() << (7 - k) for k in range(8)))
            f.crc_calc = crc15(destuffed)
            f.crc_rx = sum(take() << (14 - k) for k in range(15))
            # a stuff bit may follow the last CRC bit
            if run == 5:
                b = bits[j]; j += 1
                if b == last: raise ValueError("stuff error after CRC")
                f.nstuff += 1
            crc_delim, ack, ack_delim = bits[j], bits[j + 1], bits[j + 2]
            eof = bits[j + 3:j + 10]
            f.acked = (ack == 0)
            if f.crc_rx != f.crc_calc:
                f.error = f"CRC error rx=0x{f.crc_rx:04X} calc=0x{f.crc_calc:04X}"
            elif crc_delim != 1 or ack_delim != 1:
                f.error = "form error in delimiter"
            elif not f.acked:
                f.error = "ACK error (no dominant ACK)"
            elif any(e != 1 for e in eof[:6]):
                f.error = "form error in EOF"
            else:
                f.ok = True
            j += 10
        except ValueError as e:
            f.error = str(e)
        except IndexError:
            f.error = "truncated"
        f.end = j
        frames.append(f)
        # restart idle detection after the frame: count the recessive bits that
        # ended it (ACK delimiter + EOF), so a SOF right after intermission is seen
        i = max(j, i + 1)
        ones = 0
        k = i - 1
        while k >= 0 and bits[k] == 1 and ones < idle_needed:
            ones += 1; k -= 1
    return frames


def self_test():
    # known vector: std ID 0x123, DLC 8, 11 22 .. 88
    bits, crc = encode_frame(0x123, 0, 0, 8, bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]))
    fr = decode_stream([1] * 11 + bits + [1] * 11)
    assert len(fr) == 1 and fr[0].ok and fr[0].id == 0x123 and fr[0].crc_rx == crc, fr
    bits, crc = encode_frame(0x1ABCDE12, 1, 0, 5, bytes([0xF0, 0xE1, 0xC3, 0xA5, 0x7E]))
    fr = decode_stream([1] * 11 + bits + [1] * 11)
    assert fr[0].ok and fr[0].id == 0x1ABCDE12 and fr[0].ide == 1
    return True


if __name__ == "__main__":
    print("golden model self-test:", self_test())
    bits, crc = encode_frame(0x123, 0, 0, 8, bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]))
    print(f"STD 0x123 DLC8 -> CRC=0x{crc:04X}, {len(bits)} bits on the wire")
    print("".join(map(str, bits)))
