function [k_us, sgn, pw_home, R2] = calibrate_servos(cal_files, doPlot)
%CALIBRATE_SERVOS Fit each servo's pulse-width/angle relationship independently.
%   [k_us, sgn, pw_home, R2] = calibrate_servos(cal_files) reads one
%   calibration CSV per servo (columns: pulse_width_us, angle_deg) and
%   fits the linear model used by angle_to_pulse.m,
%       pw = pw_home(i) + sgn(i)*k_us(i)*theta
%   separately for each servo axis i — no data or fit parameters are
%   shared across servos, since each one can have its own slope, offset
%   and direction after assembly.
%
%   cal_files - 1xN cell array of CSV paths, one per servo
%               (default: {'cal_servo1.csv','cal_servo2.csv','cal_servo3.csv'})
%   doPlot    - true to plot each servo's fit for a visual sanity check
%               (default false)
%
%   Paste the printed k_us / sgn / pw_home lines directly into init_scan.m.

if nargin < 1 || isempty(cal_files)
    cal_files = {'cal_servo1.csv', 'cal_servo2.csv', 'cal_servo3.csv'};
end
if nargin < 2
    doPlot = false;
end

n = numel(cal_files);
k_us    = zeros(1,n);
sgn     = zeros(1,n);
pw_home = zeros(1,n);
R2      = zeros(1,n);
npts    = zeros(1,n);

for i = 1:n
    data = readmatrix(cal_files{i});
    pw = data(:,1);   % measured pulse width, us
    th = data(:,2);   % measured angle, deg
    npts(i) = numel(th);

    p = polyfit(th, pw, 1);   % pw = p(1)*th + p(2) — this servo's data only
    pw_home(i) = p(2);
    k_us(i)    = abs(p(1));
    sgn(i)     = sign(p(1));

    pw_fit = polyval(p, th);
    ss_res = sum((pw - pw_fit).^2);
    ss_tot = sum((pw - mean(pw)).^2);
    R2(i)  = 1 - ss_res/ss_tot;

    if R2(i) < 0.98
        warning('calibrate_servos:poorFit', ...
            'Servo %d: R^2 = %.4f — check for a mis-recorded point or slack in the rig.', i, R2(i));
    end

    if doPlot
        figure;
        scatter(th, pw, 'filled'); hold on;
        plot(th, pw_fit, '-');
        xlabel('angle (deg)'); ylabel('pulse width (us)');
        title(sprintf('Servo %d: k_{us}=%.3f us/deg, sgn=%+d, pw_{home}=%.1f us (R^2=%.5f)', ...
            i, k_us(i), sgn(i), pw_home(i), R2(i)));
        grid on;
    end
end

fprintf('Per-servo calibration (each fit independently):\n');
for i = 1:n
    fprintf('  Servo %d: n=%d pts   k_us = %.4f us/deg   sgn = %+d   pw_home = %.1f us   R^2 = %.5f\n', ...
        i, npts(i), k_us(i), sgn(i), pw_home(i), R2(i));
end
fprintf('\nPaste into init_scan.m:\n');
fprintf('k_us    = [%s];\n', strtrim(sprintf('%.4f ', k_us)));
fprintf('sgn     = [%s];\n', strtrim(sprintf('%d ', sgn)));
fprintf('pw_home = [%s];\n', strtrim(sprintf('%.1f ', pw_home)));

end
