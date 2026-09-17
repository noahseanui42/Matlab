function [pw, sat] = angle_to_pulse_lut(th_deg, cal_angle, cal_pw, pw_home, pw_min, pw_max)
%ANGLE_TO_PULSE_LUT Convert joint angles to pulse widths by interpolating
%   each servo's measured calibration table, instead of a linear formula.
%
%   Standalone alternative to Kinematics/angle_to_pulse.m - same
%   input/output shape (Nx3 th_deg in, Nx3 pw/sat out) so it could later
%   replace angle_to_pulse.m at its call sites without changing callers,
%   but kept in its own folder until the lookup-table calibration this
%   reads from (calibrate_servos_lut.m) is validated against real
%   hardware.
%
%   [pw, sat] = angle_to_pulse_lut(th_deg, cal_angle, cal_pw, pw_home, pw_min, pw_max)
%
%   th_deg   - Nx3 desired joint angles, deg
%   cal_angle, cal_pw - 1x3 cell arrays from calibrate_servos_lut.m:
%              cal_angle{i}/cal_pw{i} are servo i's measured (angle,
%              pulse width) table, sorted by angle
%   pw_home  - 1x3, pulse width used for NaN/invalid angles
%   pw_min, pw_max - 1x3, hard pulse-width limits per servo
%#codegen
N = size(th_deg,1);
pw  = zeros(N,3);
sat = false(N,3);

for k = 1:N
    for i = 1:3
        if isnan(th_deg(k,i))
            pw(k,i)  = pw_home(i);
            sat(k,i) = true;
            continue
        end
        p = round(interp1(cal_angle{i}, cal_pw{i}, th_deg(k,i), 'linear', 'extrap'));
        if p < pw_min(i)
            p = pw_min(i);  sat(k,i) = true;
        elseif p > pw_max(i)
            p = pw_max(i);  sat(k,i) = true;
        end
        pw(k,i) = p;
    end
end
end
