function [cal_angle, cal_pw, backlash_deg, cal] = calibrate_servos_lut(fwd_files, bwd_files, doPlot)
%CALIBRATE_SERVOS_LUT Build a per-servo pulse-width<->angle lookup table
%   from bidirectional calibration sweeps, fit independently per servo.
%
%   Standalone alternative to Kinematics/calibrate_servos.m - does not
%   modify or call it. Kept in its own folder so the existing linear
%   calibration pipeline (calibrate_servos.m -> init_scan.m ->
%   angle_to_pulse.m -> pulse_table.csv) stays untouched until this
%   lookup-table approach is validated against real hardware.
%
%   [cal_angle, cal_pw, backlash_deg, cal] = calibrate_servos_lut(fwd_files, bwd_files)
%
%   fwd_files, bwd_files - 1xN cell arrays of CSV paths, one per servo,
%       each with columns [pulse_width_us, angle_deg]. fwd_files is the
%       increasing-pulse-width sweep, bwd_files the decreasing sweep, both
%       using the same commanded pulse widths so points can be paired.
%       Repeated rows at the same pulse_width_us are averaged.
%   doPlot - true to plot forward/backward/averaged curves per servo
%       (default false)
%
%   Returns, per servo i:
%     cal_angle{i}, cal_pw{i} - averaged (angle, pulse width) table,
%         sorted by angle, ready for interp1 in angle_to_pulse_lut.m
%     backlash_deg(i)         - mean |forward-backward| angle gap at
%         matching commanded pulse widths, i.e. this servo's hysteresis
%     cal(i)                  - struct with the raw averaged fwd/bwd
%         curves and repeat spread (std) per point, for inspection/plots

if nargin < 3
    doPlot = false;
end

n = numel(fwd_files);
cal_angle    = cell(1,n);
cal_pw       = cell(1,n);
backlash_deg = zeros(1,n);
cal = struct('pw_fwd',{},'angle_fwd',{},'angle_fwd_std',{}, ...
             'pw_bwd',{},'angle_bwd',{},'angle_bwd_std',{});

for i = 1:n
    [pw_f, ang_f, ang_f_std] = average_repeats(fwd_files{i});
    [pw_b, ang_b, ang_b_std] = average_repeats(bwd_files{i});

    cal(i).pw_fwd = pw_f;  cal(i).angle_fwd = ang_f;  cal(i).angle_fwd_std = ang_f_std;
    cal(i).pw_bwd = pw_b;  cal(i).angle_bwd = ang_b;  cal(i).angle_bwd_std = ang_b_std;

    [pw_common, i_f, i_b] = intersect(pw_f, pw_b);
    if isempty(pw_common)
        warning('calibrate_servos_lut:noOverlap', ...
            'Servo %d: forward/backward sweeps share no common pulse widths - cannot estimate backlash, using unpaired points.', i);
        backlash_deg(i) = NaN;
        pw_avg  = [pw_f(:); pw_b(:)];
        ang_avg = [ang_f(:); ang_b(:)];
    else
        backlash_deg(i) = mean(abs(ang_f(i_f) - ang_b(i_b)));
        pw_avg  = pw_common;
        ang_avg = (ang_f(i_f) + ang_b(i_b)) / 2;
    end

    [ang_sorted, order] = sort(ang_avg);
    pw_sorted = pw_avg(order);
    [ang_sorted, uniq] = unique(ang_sorted, 'stable');
    pw_sorted = pw_sorted(uniq);

    cal_angle{i} = ang_sorted;
    cal_pw{i}    = pw_sorted;

    fprintf('Servo %d: %d table points, backlash = %.2f deg (mean fwd/bwd gap)\n', ...
        i, numel(ang_sorted), backlash_deg(i));

    if doPlot
        figure;
        plot(ang_f, pw_f, 'o-'); hold on;
        plot(ang_b, pw_b, 's-');
        plot(cal_angle{i}, cal_pw{i}, 'k.--');
        xlabel('angle (deg)'); ylabel('pulse width (us)');
        legend('forward sweep','backward sweep','averaged table','Location','best');
        title(sprintf('Servo %d calibration (backlash = %.2f deg)', i, backlash_deg(i)));
        grid on;
    end
end

end

function [pw_u, ang_mean, ang_std] = average_repeats(file)
data = readmatrix(file);
pw  = data(:,1);
ang = data(:,2);

pw_u = unique(pw);
ang_mean = zeros(size(pw_u));
ang_std  = zeros(size(pw_u));
for k = 1:numel(pw_u)
    rows = pw == pw_u(k);
    ang_mean(k) = mean(ang(rows));
    ang_std(k)  = std(ang(rows));
end
end
