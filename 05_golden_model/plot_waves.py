"""
plot_waves.py - "logic analyzer" style plots of the RTL simulation VCDs with
CAN protocol-decoder annotation (field coloured by the BSP FSM state).
Output: 00_docs/images/wave_<test>.png
"""
import os, sys, re
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
WAVES = os.path.join(ROOT, "03_tb_directed", "waves")
OUT = os.path.join(ROOT, "00_docs", "images")
os.makedirs(OUT, exist_ok=True)

STATE = {0: "INTEG", 1: "IDLE", 2: "ID", 3: "RTR/SRR", 4: "IDE", 5: "ID-B", 6: "RTR", 7: "r1",
         8: "r0", 9: "DLC", 10: "DATA", 11: "CRC", 12: "CRCdl", 13: "ACK", 14: "ACKdl", 15: "EOF",
         16: "IFS", 17: "SUSP", 18: "ERRFLAG", 19: "ERRWAIT", 20: "ERRDLM", 21: "BUSOFF"}
COLOR = {"ID": "#4C72B0", "RTR/SRR": "#8172B2", "IDE": "#8172B2", "ID-B": "#64B5CD", "RTR": "#8172B2",
         "r1": "#AAAAAA", "r0": "#AAAAAA", "DLC": "#55A868", "DATA": "#DD8452", "CRC": "#C44E52",
         "CRCdl": "#BBBBBB", "ACK": "#937860", "ACKdl": "#BBBBBB", "EOF": "#DA8BC3", "IFS": "#EEEEEE",
         "ERRFLAG": "#FF0000", "ERRWAIT": "#FF8888", "ERRDLM": "#FFBBBB", "SUSP": "#CCCCCC",
         "IDLE": "#FFFFFF", "INTEG": "#FFFFFF", "BUSOFF": "#000000"}


def parse_vcd(path):
    ids, sigs, t = {}, {}, 0
    scope = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("$scope"):
                scope.append(line.split()[2])
            elif line.startswith("$upscope"):
                scope.pop()
            elif line.startswith("$var"):
                p = line.split()
                width, code, name = int(p[2]), p[3], p[4]
                full = ".".join(scope[1:] + [name])
                ids.setdefault(code, []).append(full)
                sigs[full] = []
            elif line.startswith("#"):
                t = int(line[1:])
            elif line and line[0] in "01xzXZ" and len(line) > 1 and not line.startswith("$"):
                v, code = line[0], line[1:]
                for n in ids.get(code, []):
                    sigs[n].append((t, 0 if v in "0" else 1 if v == "1" else None))
            elif line.startswith("b"):
                v, code = line[1:].split()
                try:
                    val = int(v, 2)
                except ValueError:
                    val = None
                for n in ids.get(code, []):
                    sigs[n].append((t, val))
    return sigs


def step(ax, ch, y, t_end, scale, color="k", lw=1.2):
    if not ch:
        return
    xs, ys = [], []
    for (t, v), nxt in zip(ch, ch[1:] + [(t_end, None)]):
        vv = 0 if v is None else v
        xs += [t * scale, nxt[0] * scale]
        ys += [y + 0.7 * vv, y + 0.7 * vv]
    ax.plot(xs, ys, color=color, lw=lw, drawstyle="default")


def bits_of(ch, bit):
    return [(t, None if v is None else (v >> bit) & 1) for t, v in ch]


def plot(test, title, t0_us=None, t1_us=None):
    path = os.path.join(WAVES, f"wave_{test}.vcd")
    s = parse_vcd(path)
    # timescale 1ps in Icarus VCD for `timescale 1ns/1ps
    scale = 1e-6  # ps -> us
    t_end = max(ch[-1][0] for ch in s.values() if ch)
    bus = s["can_bus"]
    first_sof = next((t for t, v in bus if v == 0 and t > 0), 0) * scale
    if t0_us is None:
        t0_us = first_sof - 10
    if t1_us is None:
        t1_us = t0_us + 300
    fig, ax = plt.subplots(figsize=(18, 7.5))
    rows = [("CAN bus (wired-AND)", bus, "k"),
            ("node0 can_tx", bits_of(s["can_tx"], 0), "#1f77b4"),
            ("node1 can_tx", bits_of(s["can_tx"], 1), "#2ca02c"),
            ("node2 can_tx", bits_of(s["can_tx"], 2), "#9467bd"),
            ("error injection", s.get("inject_dom", []), "r"),
            ("node0 IRQ", bits_of(s["irq"], 0), "#ff7f0e")]
    ytick, ylab = [], []
    y = 0
    for name, ch, col in reversed(rows):
        step(ax, ch, y, t_end, scale, col)
        ytick.append(y + 0.35); ylab.append(name); y += 1.2
    # protocol decoder row from node0 BSP state
    st = s["dut0.u_bsp.st"]
    for (t, v), nxt in zip(st, st[1:] + [(t_end, None)]):
        name = STATE.get(v, "?")
        a, b = t * scale, nxt[0] * scale
        if b < t0_us or a > t1_us:
            continue
        ax.add_patch(Rectangle((a, y), b - a, 0.8, color=COLOR.get(name, "#DDDDDD"), ec="k", lw=0.3))
        if (b - a) > 6 and name not in ("IDLE", "INTEG"):
            ax.text((a + b) / 2, y + 0.4, name, ha="center", va="center", fontsize=7,
                    color="white" if name in ("ID", "CRC", "ERRFLAG", "BUSOFF") else "black")
    ytick.append(y + 0.4); ylab.append("node0 BSP field (decoder)"); y += 1.2
    # TEC / REC of node0
    for key, lab in (("dut0.u_bsp.tec", "node0 TEC"), ("dut1.u_bsp.rec", "node1 REC")):
        if key in s:
            for (t, v), nxt in zip(s[key], s[key][1:] + [(t_end, None)]):
                a, b = t * scale, nxt[0] * scale
                if b < t0_us or a > t1_us:
                    continue
                ax.add_patch(Rectangle((a, y), b - a, 0.8, fc="#f7f7f7", ec="k", lw=0.4))
                if b - a > 8:
                    ax.text((a + b) / 2, y + 0.4, str(v), ha="center", va="center", fontsize=8)
            ytick.append(y + 0.4); ylab.append(lab); y += 1.2
    ax.set_xlim(t0_us, t1_us); ax.set_ylim(-0.3, y)
    ax.set_yticks(ytick); ax.set_yticklabels(ylab, fontsize=9)
    ax.set_xlabel("time (µs)   —   500 kbit/s, 2 µs per bit")
    ax.set_title(title, fontsize=12, fontweight="bold")
    ax.grid(axis="x", alpha=0.3)
    plt.tight_layout()
    out = os.path.join(OUT, f"wave_{test}.png")
    plt.savefig(out, dpi=130); plt.close()
    print("wrote", out)


if __name__ == "__main__":
    plot("std_frame", "Standard data frame ID 0x123, DLC 8 — node0 transmits, node1/node2 ACK (3 nodes, 3 clocks)")
    plot("arbitration", "Bit-wise arbitration: node0 (0x123) vs node1 (0x122) — node0 loses at ID bit 0, retransmits",
         None, None)
    plot("error_inject", "Error injection in DATA field: bit error -> active error flag -> error delimiter -> automatic retransmission")
