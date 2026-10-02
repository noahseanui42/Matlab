function test_compare_tilt_runs()
% test_compare_tilt_runs — check compare_tilt_runs on synthetic repeated scans.
%
% Three magnet runs and a background on the 2026-10-02 grid. The platform tilts in
% a fixed pattern (as in run 1, up to ~2 deg) plus a random 0.3 deg per point per
% run; the field (Earth + a dipole underneath) and gravity are fixed in the world,
% so both readings tilt with the sensor. Tilt correction must cut the spread over
% runs and the dipole fit residual well down (the rest is accelerometer noise).

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'Kinematics'));
xyz = scan_grid([-50 50], [-50 50], [-675 -625], 5, 5, 3);
N = size(xyz, 1);
earth = [0.02 -0.19 0.70];
r0 = [15 -45 -890]; m = [0.3 0.1 3.3];       % about what run 1's fit found
noiseB = 5e-4; noiseA = 2e-3; jitter = 0.3;   % G, g (per-point mean), deg

rng(2);
dir_ = tempname(); mkdir(dir_);
cleanup = onCleanup(@() rmdir(dir_, 's'));
files = cell(3, 1);
for k = 1:3
    files{k} = fullfile(dir_, sprintf('magnet_tilt_run%d.csv', k));
    write_run(files{k}, xyz, earth + dipole(xyz, r0, m), noiseB, noiseA, jitter);
end
bg = fullfile(dir_, 'bg_tilt.csv');
write_run(bg, xyz, repmat(earth, N, 1), noiseB, noiseA, jitter);

C = compare_tilt_runs(fullfile(dir_, 'magnet_tilt_run*.csv'), bg, 'Plot', false);
R = C.repeat; D = C.fits;
assert(numel(C.runs) == 3 && C.n_points == N, 'wrong runs or points');
% what is left after correction is mostly accelerometer noise (2 mg here) carried into the rotation
assert(median(R.sig_vec_cor) < 0.7 * median(R.sig_vec_raw), 'correction did not reduce the spread');
assert(median(R.sig_mag_cor) < median(R.sig_mag_raw), 'magnet-field spread not reduced');
assert(abs(median(R.sig_tilt) - jitter * 0.8) < 0.15, 'tilt spread %.3f, expected about %.2f', ...
    median(R.sig_tilt), jitter * 0.8);
assert(all(D.rms_cor < 0.6 * D.rms_raw), 'correction did not improve the fits');
assert(all(sqrt(sum((D.r0_cor - r0).^2, 2)) < 10), 'corrected fit position off');
% the wildcard skipped the _tiltcorr files written by the first call
C2 = compare_tilt_runs(fullfile(dir_, 'magnet_tilt_run*.csv'), bg, 'Plot', false);
assert(numel(C2.runs) == 3, 'tiltcorr files were picked up as runs');
% no background: fits with a constant background, still runs
C3 = compare_tilt_runs({files{1}, files{2}}, '', 'Plot', false);
assert(C3.fits.fit_offset && numel(C3.runs) == 2);
fprintf('\ntest_compare_tilt_runs: all checks passed.\n');
end


function write_run(f, xyz, Bw, noiseB, noiseA, jitter)
N = size(xyz, 1);
fid = fopen(f, 'w');
fprintf(fid, ['idx,x_mm,y_mm,z_mm,Bx_G,By_G,Bz_G,Bx_std_G,By_std_G,Bz_std_G,err,t_s,n_samples,' ...
              'ax_g,ay_g,az_g,acc_pitch_deg,acc_roll_deg\n']);
s = noiseB * sqrt(20);
for i = 1:N
    % fixed pattern (pitch falls with y and x, roll falls with x, as in run 1) + random jitter
    pitch = 1.3 - 0.015 * xyz(i, 2) - 0.008 * xyz(i, 1);
    roll = -0.6 - 0.013 * xyz(i, 1);
    th = jitter * randn(); ph = 2 * pi * rand();
    R = rotm([cos(ph) sin(ph) 0], th) * rotm([0 1 0], pitch) * rotm([1 0 0], roll);
    B = (R.' * Bw(i, :).').' + noiseB * randn(1, 3);
    A = (R.' * [0; 0; -1]).' + noiseA * randn(1, 3);
    fprintf(fid, '%d,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,0,%d,20,%.6f,%.6f,%.6f,%.4f,%.4f\n', ...
        i, xyz(i, :), B, s, s, s, i, A, pitch, roll);
end
fclose(fid);
end


function R = rotm(k, deg)
k = k(:) / norm(k); t = deg2rad(deg);
K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
R = eye(3) + sin(t) * K + (1 - cos(t)) * K * K;
end


function B = dipole(P, r0, m)
d = (P - r0) / 1000; r = sqrt(sum(d.^2, 2)); u = d ./ r;
B = 1e-7 * (3 * (u * m(:)) .* u - m) ./ r.^3 * 1e4;
end
