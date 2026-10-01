"""Export the enclosure as STEP solids for SolidWorks (File > Open > .step).

Same geometry and parameters as enclosure.scad; keep the two in sync.
Origin is the outer front-left bottom corner of the box, mm units.

    python3 enclosure_step.py   ->  enclosure_box.step, enclosure_lid.step
"""
import math

import cadquery as cq

# --- Box ---
L, W, H = 200, 200, 60
wall, floor_t = 3, 3
print_corr = 0.2

# --- Floor holes (from base-plate sketch) ---
tri_side, screw_d, wire_d = 175, 8, 15
tri_R = tri_side / math.sqrt(3)
tri_cx = L / 2
tri_cy = W / 2 - (tri_R - tri_R / 2) / 2

# --- Wall holes ---
power_d, power_z = 12, 25
usb_slot_w, usb_slot_h, usb_z = 32, 14, 25

# --- Lid ---
lid_t, lip_h, lip_t, lip_clear = 3, 6, 2, 0.3


def box():
    body = (cq.Workplane("XY").box(L, W, H, centered=False)
            .cut(cq.Workplane("XY", origin=(wall, wall, floor_t))
                 .box(L - 2 * wall, W - 2 * wall, H, centered=False)))

    screws = [(tri_cx + tri_R * math.cos(math.radians(a)),
               tri_cy + tri_R * math.sin(math.radians(a))) for a in (90, 210, 330)]
    floor_cut = (cq.Workplane("XY").pushPoints(screws)
                 .circle((screw_d + print_corr) / 2)
                 .moveTo(tri_cx, tri_cy).circle((wire_d + print_corr) / 2)
                 .extrude(floor_t))
    body = body.cut(floor_cut)

    # Front wall (y = 0): power in
    power = (cq.Workplane("XZ", origin=(0, 0, 0))
             .center(L / 2, power_z)
             .circle((power_d + print_corr) / 2)
             .extrude(-wall))
    body = body.cut(power)

    # Right wall (x = L): slot for the 2 PC cables, long axis along Y
    slot = (cq.Workplane("YZ", origin=(L - wall, 0, 0))
            .center(W / 2, usb_z)
            .slot2D(usb_slot_w + print_corr, usb_slot_h + print_corr, 0)
            .extrude(wall))
    return body.cut(slot)


def lid():
    o = wall + lip_clear
    plate = cq.Workplane("XY").box(L, W, lid_t, centered=False)
    lip = (cq.Workplane("XY", origin=(o, o, lid_t))
           .box(L - 2 * o, W - 2 * o, lip_h, centered=False)
           .cut(cq.Workplane("XY", origin=(o + lip_t, o + lip_t, lid_t))
                .box(L - 2 * o - 2 * lip_t, W - 2 * o - 2 * lip_t, lip_h, centered=False)))
    return plate.union(lip)


if __name__ == "__main__":
    cq.exporters.export(box(), "enclosure_box.step")
    cq.exporters.export(lid(), "enclosure_lid.step")
