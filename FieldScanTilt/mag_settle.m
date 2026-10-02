function [waited_s, settled] = mag_settle(mag, cfg)
% mag_settle — wait after a move before reading the 1044.
%
% cfg.gyro_settle_dps = 0: a fixed wait of cfg.settle_s.
% cfg.gyro_settle_dps > 0: wait until the gyro's angular rate has stayed below
%   gyro_settle_dps for cfg.still_s, at least cfg.settle_min_s and at most
%   cfg.settle_s.
%
% Returns the time waited (s) and a flag: 1 = gyro went quiet, 0 = gyro wait
% timed out, -1 = fixed wait. Same as field_scan_tilt.wait_settled (it is called
% here, so the gyro is watched by the same code as in the Python scan).

c = py.types.SimpleNamespace(pyargs( ...
    'settle_s', cfg.settle_s, 'gyro_settle_dps', cfg.gyro_settle_dps, ...
    'still_s', cfg.still_s, 'settle_min_s', cfg.settle_min_s, 'sample_dt_s', cfg.sample_dt_s));
out = cell(mag.mod.wait_settled(mag.sensor, c));
waited_s = double(out{1});
settled = double(out{2});
end
