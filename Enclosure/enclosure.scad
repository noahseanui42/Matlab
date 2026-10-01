// ENGR489 electronics enclosure: open-top box + friction-fit lid.
// Print in PLA (non-magnetic). All dimensions in mm.
//
// Floor holes match the base-plate sketch: three Ø8 screw holes on a
// 175 mm equilateral triangle, Ø15 wire hole at the triangle centroid.
//
// Export:  openscad -D 'part="box"' -o box.stl enclosure.scad
//          openscad -D 'part="lid"' -o lid.stl enclosure.scad

part = "both";          // "box", "lid" or "both"

// --- Box ---
L = 200;                // outer length (X)
W = 200;                // outer width  (Y)
H = 60;                 // outer height of the box (without lid)
wall = 3;
floor_t = 3;
print_corr = 0.2;       // +0.2 mm hole correction for printed parts

// --- Floor holes (from base-plate sketch) ---
tri_side = 175;         // screw triangle side length
screw_d = 8;            // screw hole diameter
wire_d = 15;            // centre wire hole diameter
tri_R = tri_side / sqrt(3);                 // centroid -> screw distance
// Centre the triangle's extent in the box (flat edge toward front, y = 0)
tri_cx = L / 2;
tri_cy = W / 2 - (tri_R - tri_R / 2) / 2;

// --- Wall holes ---
power_d = 12;           // front wall: fits a 5.5x2.1 panel jack or battery leads
power_z = 25;           // height of hole centre above the bottom
usb_slot_w = 32;        // right wall: slot for 2 cables to the PC
usb_slot_h = 14;        // big enough to pass USB plugs through
usb_z = 25;

// --- Lid ---
lid_t = 3;
lip_h = 6;
lip_t = 2;
lip_clear = 0.3;        // gap between lip and inner wall for a friction fit

$fn = 64;

module screw_positions() {
    translate([tri_cx, tri_cy])
        for (a = [90, 210, 330])
            rotate(a) translate([tri_R, 0]) children();
}

module box() {
    difference() {
        cube([L, W, H]);
        translate([wall, wall, floor_t]) cube([L - 2*wall, W - 2*wall, H]);

        // Floor: 3 screw holes + centre wire hole
        screw_positions()
            translate([0, 0, -1]) cylinder(d = screw_d + print_corr, h = floor_t + 2);
        translate([tri_cx, tri_cy, -1]) cylinder(d = wire_d + print_corr, h = floor_t + 2);

        // Front wall (y = 0): power in
        translate([L/2, -1, power_z]) rotate([-90, 0, 0])
            cylinder(d = power_d + print_corr, h = wall + 2);

        // Right wall (x = L): rounded slot for the 2 PC cables
        translate([L - wall - 1, W/2, usb_z]) rotate([0, 90, 0])
            hull() for (s = [-1, 1])
                translate([0, s * (usb_slot_w - usb_slot_h) / 2, 0])
                    cylinder(d = usb_slot_h + print_corr, h = wall + 2);
    }
}

module lid() {
    // Printed top-down: plate on the bed, lip pointing up
    cube([L, W, lid_t]);
    o = wall + lip_clear;
    translate([o, o, lid_t]) difference() {
        cube([L - 2*o, W - 2*o, lip_h]);
        translate([lip_t, lip_t, -1]) cube([L - 2*o - 2*lip_t, W - 2*o - 2*lip_t, lip_h + 2]);
    }
}

if (part == "box")  box();
if (part == "lid")  lid();
if (part == "both") { box(); translate([L + 15, 0, 0]) lid(); }
