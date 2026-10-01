function cfg = scan_config()
% scan_config — every setting for the field scan in one place.
% Edit the values here, not in run_field_scan.m.

% --- robot serial link (Arduino UNO R4 Minima running delta_servo) ---
cfg.port     = "/dev/cu.usbmodem14101";  % run serialportlist("available") to find yours
cfg.baud     = 115200;                   % ignored by the R4's USB CDC, kept for the GUI's value
cfg.tcp      = [0 0 -21];                % probe offset below effector, mm (robot_config.TCP_DEFAULT)
cfg.speed_v  = 2;                        % 1..10 -> 5..50 mm/s (firmware SPEED_PER_V = 5)
cfg.move_timeout_s = 30;                 % give up if a single move takes longer than this
cfg.settle_s = 0.5;                      % wait after each move before sampling (servo jitter)
cfg.park_xyz = [0 0 -650];               % probe position at start and end of a scan, mm

% --- scan grid, PROBE (TCP) coordinates, mm ---
% Reachable on-axis range is roughly z = -550..-790 (see engr489-firmware/README.md);
% points the firmware rejects as unreachable are logged as NaN and skipped.
cfg.xr = [-50 50];   cfg.nx = 5;
cfg.yr = [-50 50];   cfg.ny = 5;
cfg.zr = [-700 -600]; cfg.nz = 5;

% --- magnetometer (Phidget 1044 via the Phidget22 Python package) ---
cfg.python      = "";       % full path to the python that has Phidget22, e.g. "/opt/anaconda3/bin/python"
                            % ("" = whatever MATLAB's pyenv already uses)
cfg.mag_serial  = 302277;   % your 1044's serial number (0 = first one found)
cfg.n_avg       = 20;       % samples averaged at each point
cfg.sample_dt_s = 0.02;     % time between samples, s

% Rotation from sensor axes to robot axes: B_robot = R * B_sensor.
% Leave as eye(3) if the 1044's x/y/z are mounted parallel to the robot's x/y/z.
cfg.R_sensor_to_robot = eye(3);

% --- output ---
cfg.out_dir = fullfile(fileparts(mfilename('fullpath')), "data");
end
