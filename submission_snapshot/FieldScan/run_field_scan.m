function csvFile = run_field_scan(label)
% run_field_scan — move the probe through the grid in scan_config.m, read
% the 1044 at every point and log to FieldScan/data/<label>_<time>.csv.
%
%   run_field_scan("baseline")   % coils OFF: Earth + servo + offset field
%   run_field_scan("coils_on")   % coils ON
%   plot_field_map(<coils_on csv>, <baseline csv>)   % baseline-subtracted map
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

% --- connect: magnetometer first so a sensor problem fails before the robot moves ---
mag = mag_open(cfg);
cleanup = onCleanup(@() mag.close());
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

fid = fopen(csvFile, "w");
fprintf(fid, "idx,x_mm,y_mm,z_mm,Bx_G,By_G,Bz_G,Bx_std_G,By_std_G,Bz_std_G,err,t_s\n");
fclose(fid);

fprintf("Scanning %d points -> %s\n", N, csvFile);
tScan = tic;
nSkipped = 0;
for k = 1:N
    e = delta_move(s, cfg, xyz(k,:));
    if e == 1
        B = nan(1,3); Bsd = nan(1,3);
        nSkipped = nSkipped + 1;
    else
        pause(cfg.settle_s);
        [B, Bsd] = mag_read(mag, cfg.n_avg, cfg.sample_dt_s);
        B   = (cfg.R_sensor_to_robot * B.').';
        Bsd = abs(cfg.R_sensor_to_robot) * Bsd.';   % approximate for non-axis-aligned R
        Bsd = Bsd.';
    end

    fid = fopen(csvFile, "a");
    fprintf(fid, "%d,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.2f\n", ...
        k, xyz(k,:), B, Bsd, e, toc(tScan));
    fclose(fid);

    eta = toc(tScan) / k * (N - k);
    fprintf("%4d/%d  [%7.1f %7.1f %7.1f]  |B| = %.4f G  e=%d  (%.0f s left)\n", ...
        k, N, xyz(k,:), norm(B), e, eta);
end

fprintf("Done in %.0f s. %d of %d points unreachable.\n", toc(tScan), nSkipped, N);
delta_move(s, cfg, cfg.park_xyz);
fprintf("Parked. Robot left ENABLED (disabling lets the arms drop).\n");
end
