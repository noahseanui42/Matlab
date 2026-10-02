function test_tilt_correct()
% test_tilt_correct — check tilt_correct on a synthetic tilting scan. No hardware.
%
% A constant field and gravity, both fixed in the world, seen by a sensor that tilts
% by 0.01 deg per mm off the axis (as FakeTiltSensor in field_scan_tilt.py). The
% corrected field must come back constant and the tilt must match.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'Kinematics'));   % scan_grid.m
xyz = scan_grid([-50 50], [-50 50], [-675 -625], 5, 5, 3);
N = size(xyz, 1);
Bw = [0.2 0.05 -0.5];
k = 0.01;                                      % deg per mm

B = zeros(N, 3); A = zeros(N, 3); tiltTrue = zeros(N, 1);
for i = 1:N
    R = roty(k * xyz(i, 1)) * rotx(-k * xyz(i, 2));
    B(i, :) = (R.' * Bw.').';
    A(i, :) = (R.' * [0; 0; -1]).';
    tiltTrue(i) = acosd(R(3, 3));
end
A(7, :) = NaN;                                 % one point with no accelerometer reading

dir_ = tempname(); mkdir(dir_);
cleanup = onCleanup(@() rmdir(dir_, 's'));
f = fullfile(dir_, 'tilt.csv');
fid = fopen(f, 'w');
fprintf(fid, 'idx,x_mm,y_mm,z_mm,Bx_G,By_G,Bz_G,Bx_std_G,By_std_G,Bz_std_G,err,t_s,n_samples,ax_g,ay_g,az_g\n');
for i = 1:N
    fprintf(fid, '%d,%.2f,%.2f,%.2f,%.8f,%.8f,%.8f,0.0005,0.0005,0.0005,0,%d,20,%.8f,%.8f,%.8f\n', ...
        i, xyz(i, :), B(i, :), i, A(i, :));
end
fclose(fid);

S = tilt_correct(f, 'Write', true);
ok = S.ok;
assert(nnz(ok) == N - 1 && ~ok(7), 'the point with no accelerometer reading should be skipped');
assert(max(abs(S.tilt_deg(ok) - tiltTrue(ok))) < 1e-4, 'tilt does not match');
% Tilting about x then y also turns the sensor very slightly about the vertical
% (second order, ~4e-5 rad here). Gravity can't show that turn, so it stays in the
% corrected field: about 1e-5 G at the corners.
err = max(max(abs(S.B_corr_G(ok, :) - Bw)));
assert(err < 3e-5, 'corrected field not constant: off by %.2g G', err);
assert(max(max(abs(S.B_raw_G(ok, :) - Bw))) > 1e-3, 'test field should be visibly tilted');

% the written CSV: corrected B in Bx_G.., raw kept, readable by the same reader
fid = fopen(S.out_file); hdr = strsplit(strtrim(fgetl(fid)), ','); fclose(fid);
assert(isequal(hdr(end-3:end), {'Bx_raw_G', 'By_raw_G', 'Bz_raw_G', 'tilt_deg'}));
M = dlmread(S.out_file, ',', 1, 0);
assert(max(max(abs(M(ok, 5:7) - S.B_corr_G(ok, :)))) < 1e-8, 'written CSV does not hold the corrected field');

% same reference from another scan, and a given vector
S2 = tilt_correct(f, 'Reference', f);
assert(max(abs(S2.tilt_deg(ok) - S.tilt_deg(ok))) < 1e-9);
S3 = tilt_correct(f, 'Reference', [0 0 -1]);
assert(max(abs(S3.tilt_deg(ok) - tiltTrue(ok))) < 1e-4);

fprintf('\ntest_tilt_correct: all checks passed.\n');
end


function R = rotx(deg)
c = cosd(deg); s = sind(deg);
R = [1 0 0; 0 c -s; 0 s c];
end


function R = roty(deg)
c = cosd(deg); s = sind(deg);
R = [c 0 s; 0 1 0; -s 0 c];
end
