"""Size of the hybrid pose correction (|send - target|) from the repo's own pose_correction.py.
Run: PYTHONDONTWRITEBYTECODE=1 python correction_magnitude.py   (needs numpy)"""
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "engr489-firmware" / "delta_app"))
import pose_correction as pc
c = pc.PoseCorrection.load()
for p in [(50, 150, -600), (50, 150, -650), (50, 150, -700), (50, 0, -650), (0, 150, -650),
          (50, 50, -650), (50, 50, -675), (50, -50, -675)]:
    off = c.predict_offset_deg(p); s = c.compensate(p)
    print(p, "shortfall deg D9/D10/D11", np.round(off, 3), "send", np.round(s, 2),
          "|send-target| %.2f mm" % np.linalg.norm(np.array(s) - p))
