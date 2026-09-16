function [bx, by, bz] = read_magnetometer()
% read_magnetometer.m — stub. The magnetometer is read separately from
% the Arduino, so this is the one function run_scan.m depends on that
% you still need to fill in for whatever interface it's actually on
% (a USB DAQ, a second serial device, direct I2C from the host, etc.).
%
% Called once per scan point, after the arm has settled at that point.
% Must return the three field components as scalars.

error('read_magnetometer:notImplemented', ...
    'Wire this up to your actual magnetometer interface before running run_scan.');

bx = NaN; by = NaN; bz = NaN; %#ok<UNRCH>
