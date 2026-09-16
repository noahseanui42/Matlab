% calibrate_servos.m — fit k_us, sgn, pw_home per servo from measured data
%
% Expects three CSV files in the current folder, one per servo, each with
% two columns and no header: pulse_width_us, measured_angle_deg.
%   cal_servo1.csv, cal_servo2.csv, cal_servo3.csv
%
% Model: pw = pw_home + sgn*k_us*theta  ->  fit pw as a linear function of
% theta, slope = sgn*k_us, intercept = pw_home.

files = {'cal_servo1.csv','cal_servo2.csv','cal_servo3.csv'};

k_us    = zeros(1,3);
sgn     = zeros(1,3);
pw_home = zeros(1,3);

for i = 1:3
    data  = readmatrix(files{i});
    pw    = data(:,1);
    theta = data(:,2);

    p = polyfit(theta, pw, 1);   % p(1) = slope, p(2) = intercept
    slope = p(1);

    k_us(i)    = abs(slope);
    sgn(i)     = sign(slope);
    pw_home(i) = p(2);

    fit_pw   = polyval(p, theta);
    resid    = pw - fit_pw;
    rmse     = sqrt(mean(resid.^2));
    maxresid = max(abs(resid));

    fprintf('Servo %d: k_us = %.4f us/deg, sgn = %+d, pw_home = %.1f us\n', ...
        i, k_us(i), sgn(i), pw_home(i));
    fprintf('  fit residual: rmse = %.2f us, max = %.2f us (n = %d points)\n', ...
        rmse, maxresid, numel(theta));

    if maxresid > 15
        fprintf('  WARNING: residual is large relative to typical servo deadband — check for a bad point near the mechanical limits.\n');
    end
end

fprintf('\nPaste into init_scan.m:\n');
fprintf('k_us    = [%.4f %.4f %.4f];\n', k_us);
fprintf('sgn     = [%+d %+d %+d];\n', sgn);
fprintf('pw_home = [%.1f %.1f %.1f];\n', pw_home);
