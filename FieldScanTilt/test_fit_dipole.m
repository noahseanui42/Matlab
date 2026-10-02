function test_fit_dipole()
% test_fit_dipole — check fit_dipole recovers a known dipole. No hardware needed.
%
% Builds a magnet scan and a no-magnet scan on the 2026-10-02 grid (x, y +-50,
% 5 x 5; z -675..-625, 3 layers) with a known dipole outside the box, a constant
% Earth field and sensor noise, then fits it with and without the baseline.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'Kinematics'));   % scan_grid.m
xyz = scan_grid([-50 50], [-50 50], [-675 -625], 5, 5, 3);
N = size(xyz, 1);

r0 = [110 -15 -700];                 % mm
m  = [-0.15 0.04 0.10];              % A m^2
earth = [-0.01 -0.14 0.72];          % G, about what the no-magnet scan reads
noise = 5e-4;                        % G, per-axis std of each point's mean

rng(1);                              % repeatable noise
Bmag = dipole(xyz, r0, m);
dir_ = tempname(); mkdir(dir_);
cleanup = onCleanup(@() rmdir(dir_, 's'));
fMag = fullfile(dir_, 'magnet.csv');
fBg  = fullfile(dir_, 'no_magnet.csv');
write_scan(fMag, xyz, Bmag + earth + noise * randn(N, 3), noise);
write_scan(fBg,  xyz, repmat(earth, N, 1) + noise * randn(N, 3), noise);

% with the baseline
F = fit_dipole(fMag, fBg, 'Plot', false, 'R', eye(3));
check(F, r0, m, 'with baseline');
assert(F.rms_G < 3 * noise, 'with baseline: residual %.5f G, expected near the noise', F.rms_G);

% no baseline: fits the Earth field as a constant
F = fit_dipole(fMag, '', 'Plot', false, 'R', eye(3));
check(F, r0, m, 'no baseline');
assert(norm(F.B0_G - earth) < 5e-3, 'no baseline: background off by %.4f G', norm(F.B0_G - earth));

% several runs and MinRange
F = fit_dipole({fMag, fMag}, fBg, 'Plot', false, 'R', eye(3), 'MinRange', 70);
check(F(1), r0, m, 'MinRange');
assert(F(1).n_used < N, 'MinRange dropped no points');

fprintf('\ntest_fit_dipole: all checks passed.\n');
end


function check(F, r0, m, what)
dr = norm(F.r0_mm - r0);
dm = norm(F.m_Am2 - m) / norm(m);
fprintf('%s: position off by %.2f mm, moment off by %.2f %%\n', what, dr, 100 * dm);
assert(dr < 2, '%s: position off by %.2f mm', what, dr);
assert(dm < 0.03, '%s: moment off by %.1f %%', what, 100 * dm);
end


function B = dipole(P, r0, m)
% Same formula as fit_dipole, written out separately: G, mm, A m^2.
d = (P - r0) / 1000;                 % m
r = sqrt(sum(d.^2, 2));
u = d ./ r;
B = 1e-7 * (3 * (u * m(:)) .* u - m) ./ r.^3 * 1e4;
end


function write_scan(f, xyz, B, sd)
N = size(xyz, 1);
fid = fopen(f, 'w');
fprintf(fid, 'idx,x_mm,y_mm,z_mm,Bx_G,By_G,Bz_G,Bx_std_G,By_std_G,Bz_std_G,err,t_s,n_samples\n');
s = sd * sqrt(20);                   % per-sample std, so std / sqrt(n) = sd
for k = 1:N
    fprintf(fid, '%d,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,0,%d,20\n', ...
        k, xyz(k, :), B(k, :), s, s, s, k);
end
fclose(fid);
end
