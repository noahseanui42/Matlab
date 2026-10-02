function F = fit_dipole(dataFiles, baselineFile, varargin)
% fit_dipole — fit a point magnetic dipole (moment + position) to a magnet scan
% and report the residuals.
%
%   F = fit_dipole(file, baselineFile)       % magnet scan minus a no-magnet scan
%   F = fit_dipole("data/*magnet*.csv", baselineFile)  % several runs: one fit each,
%                                            % plus the spread of the fits
%   F = fit_dipole(file)                     % no baseline: also fits a constant
%                                            % background field (Earth + offsets)
%
% Options (name, value):
%   'FitOffset'   also fit a constant background field B0 (G). Default: true with no
%                 baseline, false with one (the baseline already removes it).
%   'R'           sensor -> robot rotation (default: scan_config's R_sensor_to_robot)
%   'Guess'       starting magnet position [x y z] (mm). Default: grid search.
%   'Margin'      grid search reaches this far outside the scanned box (mm), 300
%   'Clearance'   the magnet can't be closer than this to a scanned point (mm), 10
%   'MinRange'    drop points closer than this to the fitted magnet and fit again
%                 (mm), 0. Use it if the near points fit badly (finite magnet size).
%   'MaxField'    drop readings with any raw component at or above this (G).
%                 Default: 0.95 x the sensor range in the .meta.json (5.6 G if none).
%   'Volume'      magnet volume (cm^3); if given, also reports Br = mu0 |m| / V
%   'Plot'        true (default): parity plot, measured vs model arrows, residuals
%   'Save'        true: save <name>_dipole.mat and PNGs next to the CSV
%
% Model, positions r (mm) relative to the dipole at r0, moment m (A m^2):
%
%   B(r) = mu0/(4 pi) * (3 (m.u) u - m) / |r|^3  [+ B0],   u = r/|r|
%
% For a trial r0 the field is linear in m (and B0), so those are solved by least
% squares; only r0 is searched (coarse grid, then fminsearch from the best few
% grid points). No toolboxes needed. Runs in MATLAB and Octave.
%
% Units: positions mm, field gauss (1 G = 100 uT), moment A m^2. Positions are the
% TARGET probe coordinates (x_mm..z_mm), so the fitted r0 is in the same frame
% (robot frame, TCP offset from scan_config). Scans with --correction off land
% short of the target off-centre, which biases the fit; hybrid-corrected scans are
% the better input. Field vectors are rotated by R before fitting, so a wrong R
% shows up as a poor fit.
%
% Residual statistics (in F and printed):
%   rms_G        RMS of the residual vector length over the points used
%   rms_xyz_G    RMS residual per component
%   rel_rms      rms_G / RMS |B measured|
%   r2           1 - SSR / sum(|B|^2)
%   noise_G      sensor noise of each point's mean (from Bx_std_G.. / sqrt(n)),
%                RMS over the points; rms_G / noise_G >> 1 means model error
%                (not sensor noise) dominates
%   *_sd         1-sigma uncertainties from the fit's covariance. They assume
%                independent residuals, so treat them as lower bounds when
%                rms_G >> noise_G.

p = inputParser;
p.addParameter('FitOffset', []);
p.addParameter('R', []);
p.addParameter('Guess', []);
p.addParameter('Margin', 300);
p.addParameter('Clearance', 10);
p.addParameter('MinRange', 0);
p.addParameter('MaxField', []);
p.addParameter('Volume', []);
p.addParameter('Plot', true);
p.addParameter('Save', false);
p.parse(varargin{:});
opt = p.Results;
if nargin < 2, baselineFile = ''; end
baselineFile = resolve_path(baselineFile);
if isempty(opt.FitOffset), opt.FitOffset = isempty(baselineFile); end
if isempty(opt.R)
    opt.R = eye(3);
    if exist('scan_config', 'file')
        cfg = scan_config();
        if isfield(cfg, 'R_sensor_to_robot'), opt.R = cfg.R_sensor_to_robot; end
    end
end

files = expand_files(dataFiles);
for k = 1:numel(files)
    Fk = fit_one(files{k}, baselineFile, opt);
    if k == 1, F = Fk; else, F(k) = Fk; end %#ok<AGROW>
end
if numel(files) > 1
    print_spread(F);
end
end


% ---------------------------------------------------------------------------
function F = fit_one(file, baselineFile, opt)
T = read_scan(file);
P = [T.x_mm T.y_mm T.z_mm];
Braw = [T.Bx_G T.By_G T.Bz_G];
N = size(P, 1);
maxField = opt.MaxField;
if isempty(maxField), maxField = 0.95 * sensor_range(file); end

% --- which points to use ---
reason = repmat({''}, N, 1);
bad = ~all(isfinite(Braw), 2);
if isfield(T, 'err'), bad = bad | T.err ~= 0; end
reason(bad) = {'unreachable or no reading'};
sat = ~bad & any(abs(Braw) >= maxField, 2);
reason(sat) = {'saturated'};

B = Braw;
Bsd = nan(N, 3);
if isfield(T, 'Bx_std_G')
    ns = 20 * ones(N, 1);
    if isfield(T, 'n_samples'), ns = max(T.n_samples, 1); end
    Bsd = [T.Bx_std_G T.By_std_G T.Bz_std_G] ./ sqrt(ns);   % std of each point's mean
end
if ~isempty(baselineFile)
    T0 = read_scan(baselineFile);
    [found, loc] = ismember(round(10 * P), round(10 * [T0.x_mm T0.y_mm T0.z_mm]), 'rows');   % to 0.1 mm
    if ~any(found), error('Baseline has no points in common with %s.', file); end
    B0 = nan(N, 3);
    B0(found, :) = [T0.Bx_G(loc(found)) T0.By_G(loc(found)) T0.Bz_G(loc(found))];
    nob = ~found & cellfun(@isempty, reason);
    reason(nob) = {'no baseline point'};
    sat0 = cellfun(@isempty, reason) & any(abs(B0) >= maxField, 2);
    reason(sat0) = {'baseline saturated'};
    B = B - B0;
    if isfield(T0, 'Bx_std_G')
        ns0 = 20 * ones(numel(T0.x_mm), 1);
        if isfield(T0, 'n_samples'), ns0 = max(T0.n_samples, 1); end
        S0 = nan(N, 3);
        S0(found, :) = [T0.Bx_std_G(loc(found)) T0.By_std_G(loc(found)) T0.Bz_std_G(loc(found))] ...
            ./ sqrt(ns0(loc(found)));
        Bsd = sqrt(Bsd.^2 + S0.^2);
    end
end
B = (opt.R * B.').';                          % sensor axes -> robot axes
Bsd = sqrt((opt.R.^2) * (Bsd.^2).').';        % exact only for axis-aligned R
use = cellfun(@isempty, reason) & all(isfinite(B), 2);
nFit = 3 + 3 + 3 * opt.FitOffset;
if nnz(use) * 3 <= nFit
    error('%s: only %d usable points, not enough for the fit.', file, nnz(use));
end

% --- fit ---
r0 = locate(P(use, :), B(use, :), opt);
[r0, theta, ssr] = refine(r0, P(use, :), B(use, :), opt.FitOffset);
if opt.MinRange > 0
    near = use & dist_to(P, r0) < opt.MinRange;
    if any(near)
        reason(near) = {sprintf('within %g mm of the magnet', opt.MinRange)};
        use = use & ~near;
        if nnz(use) * 3 <= nFit
            error('%s: MinRange leaves only %d points.', file, nnz(use));
        end
        [r0, theta, ssr] = refine(r0(:).', P(use, :), B(use, :), opt.FitOffset);
        if any(use & dist_to(P, r0) < opt.MinRange)
            warning('%s: after refitting, some points are again within MinRange of the magnet.', file);
        end
    end
end
m = theta(1:3);
Boff = zeros(3, 1);
if opt.FitOffset, Boff = theta(4:6); end

% --- residuals and statistics ---
Bmodel = dipole_field(P, r0, m) + Boff.';
E = B - Bmodel;
Eu = E(use, :);
Bu = B(use, :);
nu = nnz(use);
dof = 3 * nu - nFit;
rmsG = sqrt(mean(sum(Eu.^2, 2)));
noise = sqrt(mean(sum(Bsd(use, :).^2, 2)));   % NaN if the CSV has no std columns

% covariance of [r0; m; B0] from the numerical Jacobian at the solution
pp = [r0(:); theta(:)];
J = num_jacobian(@(q) model_vec(P(use, :), q, opt.FitOffset), pp);
s2 = ssr / max(dof, 1);
C = s2 * pinv(J.' * J);
sd = sqrt(abs(diag(C)));
mabs = norm(m);
g = m / mabs;
mabs_sd = sqrt(g.' * C(4:6, 4:6) * g);
mhat = m / mabs;

F = struct();
F.file = file;
F.baseline = baselineFile;
F.r0_mm = r0(:).';
F.r0_sd_mm = sd(1:3).';
F.m_Am2 = m(:).';
F.m_sd_Am2 = sd(4:6).';
F.m_abs_Am2 = mabs;
F.m_abs_sd_Am2 = mabs_sd;
F.m_dir = mhat(:).';
F.m_polar_deg = acosd(mhat(3));              % angle from +z
F.m_azimuth_deg = atan2d(mhat(2), mhat(1));  % from +x towards +y
F.B0_G = Boff(:).';
F.B0_sd_G = zeros(1, 3);
if opt.FitOffset, F.B0_sd_G = sd(7:9).'; end
F.Br_T = NaN;
if ~isempty(opt.Volume)
    F.Br_T = 4e-7 * pi * mabs / (opt.Volume * 1e-6);
end
F.n_used = nu;
F.n_points = N;
F.rms_G = rmsG;
F.rms_xyz_G = sqrt(mean(Eu.^2, 1));
F.max_G = max(sqrt(sum(Eu.^2, 2)));
F.rel_rms = rmsG / sqrt(mean(sum(Bu.^2, 2)));
F.r2 = 1 - sum(Eu(:).^2) / sum(Bu(:).^2);
F.noise_G = noise;
F.fit_offset = opt.FitOffset;
F.R = opt.R;
F.P_mm = P;
F.B_G = B;
F.Bmodel_G = Bmodel;
F.resid_G = E;
F.used = use;
F.excluded_reason = reason;

print_fit(F);
if opt.Plot, F.figures = plot_fit(F); else, F.figures = []; end
if opt.Save
    [d, n] = fileparts(file);
    figs = F.figures;
    F = rmfield(F, 'figures');                 % don't store figures in the .mat
    save(fullfile(d, [n '_dipole.mat']), 'F');
    F.figures = figs;
    names = {'_dipole_parity.png', '_dipole_arrows.png', '_dipole_resid.png'};
    for i = 1:numel(F.figures)
        print(F.figures(i), fullfile(d, [n names{i}]), '-dpng', '-r150');
    end
    fprintf('Saved %s_dipole.mat%s in %s\n', n, ...
        repmat(' and the PNGs', 1, ~isempty(F.figures)), d);
end
end


% ---------------------------------------------------------------------------
% Model
% ---------------------------------------------------------------------------
function A = dipole_matrix(P, r0)
% (3N x 3) matrix so that B(:) = A * m, B in G, P and r0 in mm, m in A m^2.
% mu0/(4 pi) = 1e-7 T m/A; 1 T = 1e4 G; |r|^3 in mm^3 = 1e9 x m^3  ->  1e6.
d = P - r0(:).';
r = sqrt(sum(d.^2, 2));
u = d ./ r;
K = 1e6 ./ r.^3;
N = size(P, 1);
A = zeros(3 * N, 3);
for a = 1:3
    for b = 1:3
        A((a - 1) * N + (1:N), b) = K .* (3 * u(:, a) .* u(:, b) - (a == b));
    end
end
end


function B = dipole_field(P, r0, m)
B = reshape(dipole_matrix(P, r0) * m(:), [], 3);
end


function A = design(P, r0, fitOffset)
A = dipole_matrix(P, r0);
if fitOffset
    A = [A kron(eye(3), ones(size(P, 1), 1))];
end
end


function v = model_vec(P, q, fitOffset)
v = design(P, q(1:3), fitOffset) * q(4:end);
end


function [ssr, theta] = projected(r0, P, B, fitOffset)
% Best moment (and offset) for a dipole at r0, and the residual sum of squares.
A = design(P, r0, fitOffset);
theta = A \ B(:);
e = B(:) - A * theta;
ssr = e.' * e;
end


% ---------------------------------------------------------------------------
% Search
% ---------------------------------------------------------------------------
function starts = locate(P, B, opt)
% Starting positions for the refinement: the guess, or the best few points of a
% coarse grid around the scanned box (keeping clear of the scanned points).
if ~isempty(opt.Guess)
    starts = opt.Guess(:).';
    return
end
lo = min(P, [], 1) - opt.Margin;
hi = max(P, [], 1) + opt.Margin;
step = max(hi - lo) / 30;
[gx, gy, gz] = ndgrid(lo(1):step:hi(1), lo(2):step:hi(2), lo(3):step:hi(3));
G = [gx(:) gy(:) gz(:)];
dmin = inf(size(G, 1), 1);
for i = 1:size(P, 1)
    dmin = min(dmin, sqrt(sum((G - P(i, :)).^2, 2)));
end
G = G(dmin >= max(opt.Clearance, 1e-3), :);
ssr = inf(size(G, 1), 1);
for j = 1:size(G, 1)
    ssr(j) = projected(G(j, :), P, B, opt.FitOffset);
end
[~, order] = sort(ssr);
starts = G(order(1:min(5, end)), :);
best = starts(1, :);
if any(abs(best - lo) < step / 2 | abs(best - hi) < step / 2)
    warning(['The best grid point [%.0f %.0f %.0f] mm is on the edge of the search box; ' ...
             'the magnet may be further out. Try a bigger ''Margin'' or a ''Guess''.'], best);
end
end


function [r0, theta, ssr] = refine(starts, P, B, fitOffset)
o = optimset('TolX', 1e-3, 'TolFun', 1e-12, 'MaxFunEvals', 4000, 'MaxIter', 4000, 'Display', 'off');
ssr = inf;
for k = 1:size(starts, 1)
    [rk, sk] = fminsearch(@(r) projected(r, P, B, fitOffset), starts(k, :), o);
    if sk < ssr
        r0 = rk; ssr = sk;
    end
end
[ssr, theta] = projected(r0, P, B, fitOffset);
r0 = r0(:);
end


function J = num_jacobian(f, q)
f0 = f(q);
J = zeros(numel(f0), numel(q));
for i = 1:numel(q)
    h = 1e-6 * max(1, abs(q(i)));
    dq = zeros(size(q)); dq(i) = h;
    J(:, i) = (f(q + dq) - f(q - dq)) / (2 * h);
end
end


function d = dist_to(P, r0)
d = sqrt(sum((P - r0(:).').^2, 2));
end


% ---------------------------------------------------------------------------
% Output
% ---------------------------------------------------------------------------
function print_fit(F)
[~, n] = fileparts(F.file);
fprintf('\nDipole fit: %s', n);
if ~isempty(F.baseline)
    [~, n0] = fileparts(F.baseline);
    fprintf('  minus  %s', n0);
end
fprintf('\n  points used     %d of %d', F.n_used, F.n_points);
skipped = F.excluded_reason(~F.used & ~cellfun(@isempty, F.excluded_reason));
if ~isempty(skipped)
    [u, ~, j] = unique(skipped);
    for i = 1:numel(u)
        fprintf(', %d %s', nnz(j == i), u{i});
    end
end
fprintf('\n');
fprintf('  position r0     [%8.2f %8.2f %8.2f] mm   +- [%.2f %.2f %.2f]\n', F.r0_mm, F.r0_sd_mm);
fprintf('  moment m        [%8.4f %8.4f %8.4f] A m^2 +- [%.4f %.4f %.4f]\n', F.m_Am2, F.m_sd_Am2);
fprintf('  |m|             %.4f +- %.4f A m^2, pointing %.1f deg from +z, azimuth %.1f deg\n', ...
    F.m_abs_Am2, F.m_abs_sd_Am2, F.m_polar_deg, F.m_azimuth_deg);
if isfinite(F.Br_T)
    fprintf('  Br              %.3f T (from the given volume)\n', F.Br_T);
end
if F.fit_offset
    fprintf('  background B0   [%.4f %.4f %.4f] G\n', F.B0_G);
end
fprintf('  residual RMS    %.4f G (%.2f uT), per axis [%.4f %.4f %.4f] G, max %.4f G\n', ...
    F.rms_G, 100 * F.rms_G, F.rms_xyz_G, F.max_G);
fprintf('  relative RMS    %.1f %% of RMS |B|,  R^2 = %.4f\n', 100 * F.rel_rms, F.r2);
if isfinite(F.noise_G)
    fprintf('  sensor noise    %.5f G per point, residual = %.1f x noise\n', ...
        F.noise_G, F.rms_G / F.noise_G);
end
end


function print_spread(F)
fprintf('\n%-44s %8s %8s %8s %9s %7s %7s %9s %6s\n', 'run', 'x mm', 'y mm', 'z mm', ...
    '|m| Am^2', 'polar', 'azim', 'RMS G', 'rel %');
for k = 1:numel(F)
    [~, n] = fileparts(F(k).file);
    if numel(n) > 44, n = [n(1:41) '...']; end
    fprintf('%-44s %8.2f %8.2f %8.2f %9.4f %7.1f %7.1f %9.4f %6.1f\n', n, F(k).r0_mm, ...
        F(k).m_abs_Am2, F(k).m_polar_deg, F(k).m_azimuth_deg, F(k).rms_G, 100 * F(k).rel_rms);
end
r = reshape([F.r0_mm], 3, []).';
ma = [F.m_abs_Am2];
fprintf('%-44s %8.2f %8.2f %8.2f %9.4f\n', 'mean', mean(r, 1), mean(ma));
fprintf('%-44s %8.2f %8.2f %8.2f %9.4f\n', 'std over runs', std(r, 0, 1), std(ma));
end


function figs = plot_fit(F)
[~, name] = fileparts(F.file);
u = F.used;
P = F.P_mm(u, :); B = F.B_G(u, :); M = F.Bmodel_G(u, :); E = F.resid_G(u, :);

% --- 1: parity, measured vs model per component ---
figs(1) = figure('Name', ['Dipole parity: ' name], 'Color', 'w');
c = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19];
hold on
for a = 1:3
    plot(M(:, a), B(:, a), 'o', 'Color', c(a, :), 'MarkerFaceColor', c(a, :), 'MarkerSize', 4);
end
lim = [min([B(:); M(:)]) max([B(:); M(:)])];
plot(lim, lim, 'k-');
hold off; grid on; axis equal
xlim(lim); ylim(lim);
xlabel('model (G)'); ylabel('measured (G)');
legend({'B_x', 'B_y', 'B_z', 'y = x'}, 'Location', 'northwest');
title(sprintf('Dipole fit: measured vs model, RMS %.4f G (%.1f %%)', F.rms_G, 100 * F.rel_rms));

% --- 2: arrows, measured (black) and model (red), magnet and moment ---
figs(2) = figure('Name', ['Dipole arrows: ' name], 'Color', 'w');
sp = spacing(P);
k = sp / max(prctile90(sqrt(sum(B.^2, 2))), eps);   % mm per G: typical arrow = one spacing
quiver3(P(:, 1), P(:, 2), P(:, 3), k * B(:, 1), k * B(:, 2), k * B(:, 3), 0, 'k'); hold on
quiver3(P(:, 1), P(:, 2), P(:, 3), k * M(:, 1), k * M(:, 2), k * M(:, 3), 0, 'r');
r0 = F.r0_mm;
plot3(r0(1), r0(2), r0(3), 'mp', 'MarkerSize', 14, 'MarkerFaceColor', 'm');
L = 2 * sp;
quiver3(r0(1), r0(2), r0(3), L * F.m_dir(1), L * F.m_dir(2), L * F.m_dir(3), 0, 'm', 'LineWidth', 2);
hold off; axis equal; grid on; box on
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
legend({'measured', 'dipole model', 'fitted magnet', 'moment direction'}, 'Location', 'best');
title(sprintf('Fitted dipole at [%.1f %.1f %.1f] mm, |m| = %.4f A m^2', r0, F.m_abs_Am2));
view(-35, 25);

% --- 3: residual and field vs distance from the magnet ---
figs(3) = figure('Name', ['Dipole residuals: ' name], 'Color', 'w');
d = dist_to(P, F.r0_mm);
semilogy(d, sqrt(sum(B.^2, 2)), 'ko', d, sqrt(sum(E.^2, 2)), 'r^', 'MarkerSize', 5);
if isfinite(F.noise_G)
    hold on; semilogy([min(d) max(d)], F.noise_G * [1 1], 'b--'); hold off
    legend({'|B| measured', '|residual|', 'sensor noise'}, 'Location', 'northeast');
else
    legend({'|B| measured', '|residual|'}, 'Location', 'northeast');
end
grid on
xlabel('distance from fitted magnet (mm)'); ylabel('field (G)');
title('Residual vs distance (a dipole fits worst close to a finite magnet)');
end


function s = spacing(P)
s = inf;
for a = 1:3
    d = diff(unique(P(:, a)));
    d = d(d > 1e-6);
    if ~isempty(d), s = min(s, min(d)); end
end
if ~isfinite(s), s = 10; end
end


function v = prctile90(x)
x = sort(x(isfinite(x)));
v = x(max(1, ceil(0.9 * numel(x))));
end


% ---------------------------------------------------------------------------
% Files
% ---------------------------------------------------------------------------
function files = expand_files(f)
if ischar(f), f = {f}; end
if isstring(f), f = cellstr(f); end
files = {};
for i = 1:numel(f)
    fi = char(f{i});
    if any(fi == '*' | fi == '?')
        d = dir(fi);
        if isempty(d)
            here = fileparts(mfilename('fullpath'));
            d = dir(fullfile(here, fi));
        end
        if isempty(d), error('No scan files match %s.', fi); end
        names = sort({d.name});
        folder = fileparts(fullfile(d(1).folder, d(1).name));
        for j = 1:numel(names)
            files{end + 1} = fullfile(folder, names{j}); %#ok<AGROW>
        end
    else
        files{end + 1} = resolve_path(fi); %#ok<AGROW>
    end
end
if isempty(files), error('No scan files matched.'); end
end


function rng = sensor_range(file)
% Sensor range (G) from the scan's .meta.json, 5.6 G (the 1044_0) if it isn't there.
rng = 5.6;
[d, n] = fileparts(file);
mf = fullfile(d, [n '.meta.json']);
if ~isfile(mf), return; end
try
    meta = jsondecode(fileread(mf));
    v = meta.sensor.max_field_G;
    if iscell(v), v = cell2mat(v); end
    rng = min(v);
catch
end
end


function T = read_scan(file)
% CSV -> struct of columns by header name. textscan, so it runs in MATLAB and Octave.
fid = fopen(file);
if fid < 0, error('Cannot open %s', file); end
names = strsplit(strtrim(fgetl(fid)), ',');
C = textscan(fid, repmat('%f', 1, numel(names)), 'Delimiter', ',');
fclose(fid);
T = struct();
for i = 1:numel(names)
    T.(names{i}) = C{i};
end
end


function f = resolve_path(f)
% Accept a path relative to the current folder or to this script's folder
% (FieldScan), so "data/....csv" works from anywhere.
f = char(f);
if isempty(f) || isfile(f), return; end
here = fileparts(mfilename('fullpath'));
alt = fullfile(here, f);
if isfile(alt), f = alt; return; end
error('Cannot find %s\nLooked in %s and in %s.', f, pwd, here);
end
