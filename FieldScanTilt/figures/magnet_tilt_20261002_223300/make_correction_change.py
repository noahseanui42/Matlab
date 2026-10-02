"""Correction-change figure for magnet_tilt_20261002_223300: tilt per point and how much
the tilt correction changes Bx, By, Bz (reads the _tiltcorr.csv written by tilt_correct.m)."""
import csv, math
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm

from pathlib import Path
D = str(Path(__file__).resolve().parents[2]) + "/"   # FieldScanTilt/
rows = list(csv.DictReader(open(D + "data/magnet_tilt_20261002_223300_tiltcorr.csv")))
g = lambda k: np.array([float(r[k]) for r in rows])
x, y, z = g("x_mm"), g("y_mm"), g("z_mm")
raw = np.c_[g("Bx_raw_G"), g("By_raw_G"), g("Bz_raw_G")]
cor = np.c_[g("Bx_G"), g("By_G"), g("Bz_G")]
tilt = g("tilt_deg")
d = (cor - raw) * 1000            # mG
xs, ys, zs = np.unique(x), np.unique(y), np.unique(z)[::-1]

seq = LinearSegmentedColormap.from_list("seq", ["#cde2fb", "#6da7ec", "#256abf", "#0d366b"])
div = LinearSegmentedColormap.from_list("div", ["#184f95", "#6da7ec", "#f0efec", "#ec8a89", "#b8302f"])
ink, muted, grid = "#2b2b29", "#6b6a66", "#d9d8d4"
plt.rcParams.update({"font.size": 9, "text.color": ink, "axes.labelcolor": muted,
                     "xtick.color": muted, "ytick.color": muted, "axes.edgecolor": grid})

cols = [("Tilt vs centre point", "deg", tilt, seq, None),
        ("Change in field |ΔB|", "mG", np.linalg.norm(d, axis=1), seq, None),
        ("Change in Bx", "mG", d[:, 0], div, "div"),
        ("Change in By", "mG", d[:, 1], div, "div"),
        ("Change in Bz", "mG", d[:, 2], div, "div")]
fig, ax = plt.subplots(len(zs), len(cols), figsize=(16, 9.2), constrained_layout=True)
fig.patch.set_facecolor("#fcfcfb")
for j, (name, unit, v, cmap, kind) in enumerate(cols):
    if kind == "div":
        m = np.nanmax(np.abs(v)); norm = TwoSlopeNorm(0, -m, m)
    else:
        norm = plt.Normalize(0, np.nanmax(v))
    for i, zz in enumerate(zs):
        M = np.full((len(ys), len(xs)), np.nan)
        on = np.abs(z - zz) < 0.05
        for xv, yv, vv in zip(x[on], y[on], v[on]):
            M[np.argmin(abs(ys - yv)), np.argmin(abs(xs - xv))] = vv
        a = ax[i, j]
        im = a.imshow(M, origin="lower", cmap=cmap, norm=norm,
                      extent=[xs[0] - 12.5, xs[-1] + 12.5, ys[0] - 12.5, ys[-1] + 12.5])
        a.set_xticks([-50, 0, 50]); a.set_yticks([-50, 0, 50])
        if i == 0: a.set_title(name, fontsize=11, color=ink, pad=8)
        if j == 0: a.set_ylabel(f"z = {zz:.0f} mm\ny (mm)", color=ink)
        if i == len(zs) - 1: a.set_xlabel("x (mm)")
    cb = fig.colorbar(im, ax=ax[:, j], shrink=0.6, location="bottom", pad=0.02)
    cb.set_label(unit); cb.outline.set_edgecolor(grid)
fig.suptitle("magnet_tilt_20261002_223300: what the tilt correction changes (corrected − raw, reference = grid centre)",
             fontsize=12, color=ink)
fig.savefig(D + "figures/magnet_tilt_20261002_223300/correction_change.png", dpi=130, facecolor=fig.get_facecolor())
print("tilt deg: median %.2f max %.2f" % (np.median(tilt), tilt.max()))
print("|dB| mG: median %.1f max %.1f; rel to |B| max %.1f%%" % (np.median(np.linalg.norm(d,axis=1)), np.linalg.norm(d,axis=1).max(), 100*(np.linalg.norm(d,axis=1)/np.linalg.norm(raw,axis=1)).max()))
for k, n in enumerate("xyz"): print(f"dB{n} mG range {d[:,k].min():.1f} .. {d[:,k].max():.1f}")
print("max |B| change (should be 0): %.2e G" % np.abs(np.linalg.norm(cor,axis=1)-np.linalg.norm(raw,axis=1)).max())
