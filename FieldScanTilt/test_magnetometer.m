function test_magnetometer()
% test_magnetometer — check MATLAB can read the 1044 (field, tilt and gyro)
% before running a scan.
% Rotate the board while this runs: Bx/By/Bz should change, |B| should stay
% roughly constant (~0.55-0.6 G, the Earth's field in Wellington, coils off),
% pitch/roll should follow the tilt and |a| should stay at about 1 g. Hold it
% still and the gyro rate should drop to its noise level.

cfg = scan_config();
mag = mag_open(cfg);
cleanup = onCleanup(@() mag.sensor.close());

info = mag.info;
if isfield(info, 'max_field_G')
    fprintf("Connected. Sensor range: +/-%.2f G per axis (1 G = 100 uT).\n", max(info.max_field_G));
end
fprintf("Orientation filter: %s.\n", info.algorithm);
fprintf("%8s %8s %8s %8s | %8s %8s | %8s %8s | %7s %7s\n", "Bx", "By", "Bz", "|B| G", ...
    "pitch", "roll", "a-pitch", "a-roll", "|a| g", "gyro");
for k = 1:20
    [B, ~, r] = mag_read(mag, 5, cfg.sample_dt_s);
    fprintf("%8.4f %8.4f %8.4f %8.4f | %8.3f %8.3f | %8.3f %8.3f | %7.4f %7.3f\n", ...
        B, norm(B), r.pitch_deg, r.roll_deg, r.acc_pitch_deg, r.acc_roll_deg, ...
        norm(r.a_g), r.gyro_rms_dps);
end
fprintf("pitch/roll: board filter (deg); a-pitch/a-roll: from gravity (deg); gyro: RMS deg/s.\n");
end
