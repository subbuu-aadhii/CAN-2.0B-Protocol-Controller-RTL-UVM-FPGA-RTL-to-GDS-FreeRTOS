"""render_text.py <in.txt> <out.png> "<title>"  - terminal-style image of a text report"""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
src, out, title = sys.argv[1], sys.argv[2], sys.argv[3]
lines = open(src, encoding="utf-8", errors="replace").read().rstrip("\n").split("\n")
w = max(len(l) for l in lines)
fig = plt.figure(figsize=(max(8, w * 0.085), 0.24 * len(lines) + 0.8), facecolor="#0c0c0c")
fig.text(0.01, 1 - 0.18 / (0.24 * len(lines) + 0.8), title, color="#7fd4ff", family="monospace",
         fontsize=11, weight="bold", va="top")
fig.text(0.01, 1 - 0.55 / (0.24 * len(lines) + 0.8), "\n".join(lines), color="#e6e6e6",
         family="monospace", fontsize=10, va="top", linespacing=1.35)
plt.savefig(out, dpi=130, facecolor=fig.get_facecolor())
print("wrote", out)
