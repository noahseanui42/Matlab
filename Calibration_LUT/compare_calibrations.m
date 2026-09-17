function res = compare_calibrations(holdout_files, k_us, sgn, pw_home, cal_angle, cal_pw, pw_min, pw_max)
%COMPARE_CALIBRATIONS Residuals of the Phase 1 linear fit vs the Phase 2 lookup
%   table on held-out points, per servo. This is the step-12 comparison in
%   HANDOFF_calibration.md - the number that decides whether Calibration_LUT
%   gets adopted.
%
%   res = compare_calibrations(holdout_files, k_us, sgn, pw_home, cal_angle, cal_pw, pw_min, pw_max)
%
%   holdout_files - 1xN cell array, one CSV per servo, columns
%                   [pulse_width_us, angle_deg]: the pulse width actually
%                   commanded (sketch 'P<us>' command) and the angle read off
%                   the camera. These points must NOT have been used in either
%                   fit, or the comparison is meaningless.
%   k_us, sgn, pw_home     - Phase 1 output of Kinematics/calibrate_servos.m
%   cal_angle, cal_pw      - Phase 2 output of calibrate_servos_lut.m
%   pw_min, pw_max         - 1xN pulse limits from init_scan.m
%
%   Two residuals are reported per method and servo:
%
%   angle-domain (deg): predict the angle each method implies for the
%       commanded pulse width, subtract from the measured angle. Linear uses
%       theta = sgn*(pw - pw_home)/k_us; the table inverts (cal_pw -> cal_angle)
%       with interp1. This is the "X deg residual" figure for the report.
%
%   pulse-domain (us): run the measured angle through the real production
%       converters (angle_to_pulse.m and angle_to_pulse_lut.m) and subtract
%       the pulse width that was actually sent. Same information, but it
%       exercises the exact code path the scan pipeline uses, rounding and
%       saturation included.
%
%   res is a struct array per servo: n, lin_rms_deg, lin_max_deg,
%   lut_rms_deg, lut_max_deg, lin_rms_us, lut_rms_us, and per-point columns
%   pw_cmd, angle_meas, lin_resid_deg, lut_resid_deg.

n = numel(holdout_files);
if ~exist('angle_to_pulse', 'file')
    kin = fullfile(fileparts(mfilename('fullpath')), '..', 'Kinematics');
    if exist(fullfile(kin, 'angle_to_pulse.m'), 'file')
        addpath(kin);
    else
        error('compare_calibrations:noKinematics', ...
            'angle_to_pulse.m not on the path and not found at %s.', kin);
    end
end

res = struct('n', {}, 'lin_rms_deg', {}, 'lin_max_deg', {}, 'lut_rms_deg', {}, 'lut_max_deg', {}, ...
             'lin_rms_us', {}, 'lut_rms_us', {}, ...
             'pw_cmd', {}, 'angle_meas', {}, 'lin_resid_deg', {}, 'lut_resid_deg', {});

for i = 1:n
    data = readmatrix(holdout_files{i});
    pw_cmd  = data(:,1);
    ang_meas = data(:,2);
    m = numel(pw_cmd);

    % --- angle-domain: what angle does each method claim this pulse width gives?
    ang_lin = sgn(i) * (pw_cmd - pw_home(i)) / k_us(i);
    [pw_tab, order] = sort(cal_pw{i});            % table sorted by pulse for the inverse lookup
    ang_tab = cal_angle{i}(order);
    [pw_tab, u] = unique(pw_tab, 'stable'); ang_tab = ang_tab(u);
    ang_lut = interp1(pw_tab, ang_tab, pw_cmd, 'linear', 'extrap');

    r_lin = ang_meas - ang_lin;
    r_lut = ang_meas - ang_lut;

    % --- pulse-domain: through the production converters, this servo's column only
    th = nan(m, 3); th(:, i) = ang_meas;
    pw_lin_full = angle_to_pulse(th, k_us, pw_home, sgn, pw_min, pw_max);
    pw_lut_full = angle_to_pulse_lut(th, cal_angle, cal_pw, pw_home, pw_min, pw_max);
    e_lin_us = pw_cmd - pw_lin_full(:, i);
    e_lut_us = pw_cmd - pw_lut_full(:, i);

    r = struct('n', m, ...
        'lin_rms_deg', rms(r_lin), 'lin_max_deg', max(abs(r_lin)), ...
        'lut_rms_deg', rms(r_lut), 'lut_max_deg', max(abs(r_lut)), ...
        'lin_rms_us', rms(e_lin_us), 'lut_rms_us', rms(e_lut_us), ...
        'pw_cmd', pw_cmd, 'angle_meas', ang_meas, ...
        'lin_resid_deg', r_lin, 'lut_resid_deg', r_lut);
    res(i) = r; %#ok<AGROW>
end

fprintf('\nHeld-out residuals, per servo (linear = Phase 1, table = Phase 2):\n');
fprintf('  %-7s %-4s %12s %12s %12s %12s %10s %10s\n', 'servo', 'n', ...
    'lin RMS deg', 'lin max deg', 'tab RMS deg', 'tab max deg', 'lin RMS us', 'tab RMS us');
for i = 1:n
    r = res(i);
    fprintf('  %-7d %-4d %12.3f %12.3f %12.3f %12.3f %10.1f %10.1f\n', i, r.n, ...
        r.lin_rms_deg, r.lin_max_deg, r.lut_rms_deg, r.lut_max_deg, r.lin_rms_us, r.lut_rms_us);
end
fprintf('\n');
for i = 1:n
    r = res(i);
    if r.lut_rms_deg < r.lin_rms_deg
        fprintf('  Servo %d: table beats linear by %.3f deg RMS (%.0f%% lower).\n', i, ...
            r.lin_rms_deg - r.lut_rms_deg, 100 * (1 - r.lut_rms_deg / r.lin_rms_deg));
    else
        fprintf('  Servo %d: linear is as good or better (table %.3f vs linear %.3f deg RMS).\n', i, ...
            r.lut_rms_deg, r.lin_rms_deg);
    end
end
fprintf(['\nJudge the gap against the rig''s own noise: the per-point repeat spread\n' ...
         '(calibrate_servos_lut.m cal(i).angle_*_std) and the measured backlash. A\n' ...
         'difference smaller than those is not evidence for either method.\n']);
end
