function test_magnetometer()
% test_magnetometer — check MATLAB can read the 1044 before running a scan.
% Rotate the board while this runs: Bx/By/Bz should change, |B| should stay
% roughly constant (~0.55-0.6 G, the Earth's field in Wellington, coils off).

cfg = scan_config();
mag = mag_open(cfg);
cleanup = onCleanup(@() mag.close());

Bmax = cellfun(@double, cell(mag.getMaxMagneticField()));
fprintf("Connected. Sensor range: +/-%.2f G per axis (1 G = 100 uT).\n", max(Bmax));
fprintf("%8s %8s %8s %8s   (gauss)\n", "Bx", "By", "Bz", "|B|");
for k = 1:20
    B = mag_read(mag, 5, cfg.sample_dt_s);
    fprintf("%8.4f %8.4f %8.4f %8.4f\n", B, norm(B));
end
end
