function cfg = scan_config()
% scan_config — every setting for the field scan (with tilt) in one place.
% Edit the values here, not in run_field_scan.m.

% --- robot serial link (Arduino UNO R4 Minima running delta_servo) ---
cfg.port     = "/dev/cu.usbmodem14101";  % run serialportlist("available") to find yours
cfg.baud     = 115200;                   % ignored by the R4's USB CDC, kept for the GUI's value
cfg.tcp      = [0 0 -21];                % probe offset below effector, mm (robot_config.TCP_DEFAULT)
cfg.speed_v  = 2;                        % 1..10 -> 5..50 mm/s (firmware SPEED_PER_V = 5)
cfg.move_timeout_s = 30;                 % give up if a single move takes longer than this
cfg.settle_s = 5.0;                      % wait after each move before sampling, s (the arms swing
                                         % for a few s); with gyro_settle_dps > 0, the LONGEST wait
cfg.park_xyz = [0 0 -650];               % probe position at start and end of a scan, mm

% --- scan grid, PROBE (TCP) coordinates, mm ---
% Reachable on-axis range is roughly z = -550..-790 (see engr489-firmware/README.md);
% points the firmware rejects as unreachable are logged as NaN and skipped.
cfg.xr = [-50 50];   cfg.nx = 5;
cfg.yr = [-50 50];   cfg.ny = 5;
cfg.zr = [-700 -600]; cfg.nz = 5;

% --- sensor (Phidget 1044 via the Phidget22 Python package, Spatial channel) ---
% Every reading holds field, acceleration and gyro from the same instant.
cfg.python      = "";       % full path to the python that has Phidget22, e.g. "/opt/anaconda3/bin/python"
                            % ("" = whatever MATLAB's pyenv already uses)
cfg.mag_serial  = 302277;   % your 1044's serial number (0 = first one found)
cfg.n_avg       = 20;       % samples averaged at each point
cfg.sample_dt_s = 0.02;     % time between samples, s

% --- tilt and gyro (same options as field_scan_tilt.py) ---
cfg.algorithm       = "imu";  % board's filter for pitch/roll: "imu", "ahrs" (also uses the
                              % magnetometer, which the magnet disturbs) or "none"
cfg.gyro_settle_dps = 0;      % 0: fixed settle_s wait; > 0: wait until the gyro rate stays
                              % below this (deg/s) for still_s
cfg.still_s         = 1.0;    % gyro mode: how long the rate must stay below gyro_settle_dps, s
cfg.settle_min_s    = 0.5;    % gyro mode: shortest wait, s
cfg.zero_gyro       = true;   % zero the gyro at the park position before the scan

% Rotation from sensor axes to robot axes: B_robot = R * B_sensor.
% Leave as eye(3) if the 1044's x/y/z are mounted parallel to the robot's x/y/z.
% The CSVs stay in SENSOR axes (field and acceleration); tilt_correct, plot_tilt,
% plot_field_layers, plot_field_arrows3d and fit_dipole apply R when they plot.
cfg.R_sensor_to_robot = eye(3);

% --- output ---
cfg.out_dir = fullfile(fileparts(mfilename('fullpath')), "data");
end
