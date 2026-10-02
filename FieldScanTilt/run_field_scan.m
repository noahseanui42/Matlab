function csvFile = run_field_scan(label)
% run_field_scan — move the probe through the grid in scan_config.m, read
% the 1044 at every point (field + pitch/roll + gyro) and log to
% FieldScanTilt/data/<label>_<time>.csv.
%
%   run_field_scan("baseline")   % coils OFF: Earth + servo + offset field
%   run_field_scan("coils_on")   % coils ON
%   plot_field_map(<coils_on csv>, <baseline csv>)   % baseline-subtracted map
%   plot_tilt(<csv>)                                 % tilt and gyro per point
%
% The CSV has the same columns as field_scan_tilt.py's (field_scan.py's 19, then
% the tilt columns), so every script here reads both. The field and the
% acceleration are written in SENSOR axes; the plots apply R_sensor_to_robot.
% There is no position correction here: sent_x_mm..sent_z_mm = the target.
%
% Rows are written as they are measured, so a scan stopped with Ctrl+C
% keeps everything up to that point. Close the Python GUI before running.

if nargin < 1, label = "scan"; end
cfg = scan_config();
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, "..", "Kinematics"));   % scan_grid.m

xyz = scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz);   % serpentine order
N = size(xyz, 1);

if ~isfolder(cfg.out_dir), mkdir(cfg.out_dir); end
csvFile = fullfile(cfg.out_dir, sprintf("%s_%s.csv", label, ...
    string(datetime("now", "Format", "yyyyMMdd_HHmmss"))));

% --- connect: sensor first so a sensor problem fails before the robot moves ---
mag = mag_open(cfg);
cleanup = onCleanup(@() mag.sensor.close());
s = delta_connect(cfg);

st = delta_status(s, 2);
if st.en == 0
    fprintf("\nRobot is DISABLED. Enabling snaps the arms to their last commanded pose.\n");
    fprintf("Support the arms near flat and switch servo power ON.\n");
    input("Press Enter to enable (Ctrl+C to abort)... ", "s");
    delta_send(s, '8', struct('enable', 0));   % enable:0 means ENABLE (inverted)
    pause(0.5);
    flush(s, "input");
    st = delta_status(s, 2);
    if isempty(st) || st.en == 0
        error("Robot did not enable.");
    end
end

fprintf("Moving to park position [%g %g %g]...\n", cfg.park_xyz);
if delta_move(s, cfg, cfg.park_xyz) == 1
    error("Park position [%g %g %g] is unreachable. Change cfg.park_xyz.", cfg.park_xyz);
end
if cfg.zero_gyro
    fprintf("Zeroing the gyro at the park position (keep the robot still)...\n");
    pause(cfg.settle_s);
    mag.sensor.zero_gyro();
end

cols = ["idx" "x_mm" "y_mm" "z_mm" "Bx_G" "By_G" "Bz_G" "Bx_std_G" "By_std_G" "Bz_std_G" ...
        "err" "t_s" "n_samples" "deg1" "deg2" "deg3" "sent_x_mm" "sent_y_mm" "sent_z_mm" ...
        "ax_g" "ay_g" "az_g" "ax_std_g" "ay_std_g" "az_std_g" ...
        "gyro_rms_dps" "gyro_max_dps" "settle_s" "settled" ...
        "pitch_deg" "roll_deg" "acc_pitch_deg" "acc_roll_deg" ...
        "spatial_t_first_ms" "spatial_t_last_ms"];
fid = fopen(csvFile, "w");
fprintf(fid, "%s\n", strjoin(cols, ","));
fclose(fid);

fprintf("Scanning %d points -> %s\n", N, csvFile);
tScan = tic;
nSkipped = 0;
nUnsettled = 0;
for k = 1:N
    [e, st] = delta_move(s, cfg, xyz(k,:));
    deg = nan(1, 3);
    if isfield(st, 'deg'), deg = reshape(double(st.deg), 1, []); end
    if e == 1
        B = nan(1,3); Bsd = nan(1,3);
        r = struct('n', 0, 'a_g', nan(1,3), 'a_std_g', nan(1,3), 'gyro_rms_dps', NaN, ...
            'gyro_max_dps', NaN, 'pitch_deg', NaN, 'roll_deg', NaN, 'acc_pitch_deg', NaN, ...
            'acc_roll_deg', NaN, 't_first_ms', NaN, 't_last_ms', NaN);
        waited = 0; settled = -1;
        nSkipped = nSkipped + 1;
    else
        [waited, settled] = mag_settle(mag, cfg);
        nUnsettled = nUnsettled + (settled == 0);
        [B, Bsd, r] = mag_read(mag, cfg.n_avg, cfg.sample_dt_s);
    end

    fid = fopen(csvFile, "a");
    fprintf(fid, "%d,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.2f,%d,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f," + ...
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.2f,%d,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f\n", ...
        k, xyz(k,:), B, Bsd, e, toc(tScan), r.n, deg, xyz(k,:), ...
        r.a_g, r.a_std_g, r.gyro_rms_dps, r.gyro_max_dps, waited, settled, ...
        r.pitch_deg, r.roll_deg, r.acc_pitch_deg, r.acc_roll_deg, r.t_first_ms, r.t_last_ms);
    fclose(fid);

    pr = [r.pitch_deg r.roll_deg];
    if ~all(isfinite(pr)), pr = [r.acc_pitch_deg r.acc_roll_deg]; end
    eta = toc(tScan) / k * (N - k);
    fprintf("%4d/%d  [%7.1f %7.1f %7.1f]  |B| = %.4f G  pitch %7.3f  roll %7.3f deg  " + ...
        "gyro %.2f deg/s  settle %.1f s  e=%d  (%.0f s left)\n", ...
        k, N, xyz(k,:), norm(B), pr, r.gyro_rms_dps, waited, e, eta);
end

fprintf("Done in %.0f s. %d of %d points unreachable", toc(tScan), nSkipped, N);
if cfg.gyro_settle_dps > 0
    fprintf(", %d where the gyro never went quiet", nUnsettled);
end
fprintf(".\n");
delta_move(s, cfg, cfg.park_xyz);
fprintf("Parked. Robot left ENABLED (disabling lets the arms drop).\n");
end
