function [Bmean, Bstd, r] = mag_read(mag, n, dt_s)
% mag_read — average the next n readings of the 1044 (taken dt_s apart) into one
% point: field, acceleration, gyro, pitch and roll, all from the same readings.
%
%   [Bmean, Bstd]    = mag_read(mag, n, dt_s)   % field only, 1x3, gauss, sensor axes
%   [Bmean, Bstd, r] = mag_read(mag, n, dt_s)   % r also holds the tilt and gyro:
%
%   r.n                    readings averaged (unknown/out-of-range ones are dropped)
%   r.a_g, r.a_std_g       mean and std of the acceleration, 1x3, g, sensor axes
%   r.gyro_rms_dps         RMS angular rate while sampling, deg/s
%   r.gyro_max_dps         largest angular rate while sampling, deg/s
%   r.pitch_deg, roll_deg  mean pitch and roll from the board's IMU/AHRS filter
%                          (NaN with algorithm "none" or if the board refused it)
%   r.acc_pitch_deg, acc_roll_deg   pitch and roll from the mean acceleration
%   r.t_first_ms, t_last_ms         the 1044's timestamps of the first and last reading
%
% mag comes from mag_open. The averaging is field_scan_tilt.read_point, the same
% code the Python scan uses.

p = mag.mod.read_point(mag.sensor, int32(n), dt_s);
Bmean = vec(p.get('B'));
Bstd  = vec(p.get('Bsd'));
acc_pr = vec(p.get('acc_pr'));
ts = vec(p.get('ts'));
r = struct( ...
    'n',             double(p.get('n')), ...
    'a_g',           vec(p.get('a')), ...
    'a_std_g',       vec(p.get('asd')), ...
    'gyro_rms_dps',  double(p.get('g_rms')), ...
    'gyro_max_dps',  double(p.get('g_max')), ...
    'pitch_deg',     double(p.get('pitch')), ...
    'roll_deg',      double(p.get('roll')), ...
    'acc_pitch_deg', acc_pr(1), ...
    'acc_roll_deg',  acc_pr(2), ...
    't_first_ms',    ts(1), ...
    't_last_ms',     ts(2));
end

function v = vec(x)
v = cellfun(@double, cell(x));
end
