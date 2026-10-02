function C = compare_tilt_runs(magnetFiles, bgFile, varargin)
% compare_tilt_runs — repeatability and dipole-fit accuracy of repeated tilt scans,
% raw against tilt-corrected, on the same data.
%
%   C = compare_tilt_runs("data/magnet_tilt_*.csv", "data/bg_tilt_*.csv")
%   C = compare_tilt_runs(["data/run1.csv" "data/run2.csv"], "data/bg.csv")
%   C = compare_tilt_runs("data/magnet_tilt_*.csv")          % no background yet
%   C = compare_tilt_runs(..., 'Save', true)                 % .mat + PNGs in the data folder
%   C = compare_tilt_runs(..., 'Verbose', true)              % also print every fit
%
% Inputs are field_scan_tilt.py CSVs, all on the same grid. *_tiltcorr.csv files
% matched by a wildcard are ignored; the corrected files are (re)made here with
% tilt_correct, every run and the background to the SAME reference (the
% background's centre point, or the first run's if there is no background).
%
% What it compares, raw vs tilt-corrected:
%   1. Repeatability: at each point, the spread over runs of the field VECTOR
%      sigma_vec = sqrt(var(Bx) + var(By) + var(Bz)), and of the magnet's own field
%      |B - B_background|. (|B| itself doesn't change under a rotation, so its spread
%      can't show tilt; that is what compare_runs.m measures.) The sensor noise of
%      each point's mean is shown for reference.
%   2. Tilt repeatability: spread over runs of the tilt at each point (deg), and of
%      acc_pitch_deg / acc_roll_deg. Small = the platform tilts the same way every run.
%   3. Accuracy: fit_dipole on each run (minus the background): residual RMS, fitted
%      magnet position and moment, and their spread over runs.
%
% Returns struct C with the per-point numbers and a summary; prints a table.

p = inputParser;
p.addParameter('Plot', true);
p.addParameter('Save', false);
p.addParameter('Verbose', false);
p.parse(varargin{:});
opt = p.Results;
if nargin < 2, bgFile = ''; end
here = fileparts(mfilename('fullpath'));
if ~exist('fit_dipole', 'file')
    addpath(fullfile(here, '..', 'FieldScan'));
end

runs = expand_files(magnetFiles, here);
haveBg = ~isempty(bgFile) && ~(ischar(bgFile) && isempty(strtrim(bgFile))) ...
    && ~(isstring(bgFile) && all(strlength(bgFile) == 0));
if haveBg
    bg = expand_files(bgFile, here);
    if numel(bg) ~= 1, error('Expected one background file, got %d.', numel(bg)); end
    bg = bg{1};
    runs = runs(~strcmp(runs, bg));
end
nRun = numel(runs);
if nRun < 1, error('No magnet runs.'); end

% --- tilt-correct everything to one reference ---
if haveBg, ref = bg; else, ref = runs{1}; end
quiet = @(cmd) run_quiet(cmd, opt.Verbose);
S = cell(nRun, 1);
for k = 1:nRun
    S{k} = quiet(@() tilt_correct(runs{k}, 'Reference', ref, 'Write', true));
end
if haveBg
    Sbg = quiet(@() tilt_correct(bg, 'Reference', ref, 'Write', true));
end

% --- points present (and valid) in every run and the background ---
key = @(P) round(10 * P);
K = key(S{1}.P_mm(S{1}.ok, :));
for k = 2:nRun
    K = intersect(K, key(S{k}.P_mm(S{k}.ok, :)), 'rows');
end
if haveBg, K = intersect(K, key(Sbg.P_mm(Sbg.ok, :)), 'rows'); end
if isempty(K), error('The runs have no valid points in common.'); end
N = size(K, 1);
P = K / 10;

[Braw, Bcor, tilt, pitch, roll, noise] = deal(nan(N, 3, nRun), nan(N, 3, nRun), nan(N, nRun), ...
    nan(N, nRun), nan(N, nRun), nan(N, nRun));
for k = 1:nRun
    [~, loc] = ismember(K, key(S{k}.P_mm), 'rows');
    Braw(:, :, k) = S{k}.B_raw_G(loc, :);
    Bcor(:, :, k) = S{k}.B_corr_G(loc, :);
    tilt(:, k) = S{k}.tilt_deg(loc);
    pitch(:, k) = S{k}.acc_pitch_deg(loc);
    roll(:, k) = S{k}.acc_roll_deg(loc);
    T = read_scan(S{k}.file);
    sd = [T.Bx_std_G T.By_std_G T.Bz_std_G];
    nsamp = 20 * ones(numel(T.x_mm), 1);
    if isfield(T, 'n_samples'), nsamp = max(T.n_samples, 1); end
    noise(:, k) = sqrt(sum(sd(loc, :).^2, 2) ./ nsamp(loc));   % |noise| of the point's mean
end
noisePt = sqrt(mean(noise.^2, 2));          % noise of one point's mean, |vector|, G

% --- 1. repeatability ---
R = struct();
if nRun >= 2
    R.sig_vec_raw = sqrt(sum(var(Braw, 0, 3), 2));      % N x 1, G
    R.sig_vec_cor = sqrt(sum(var(Bcor, 0, 3), 2));
    R.sig_xyz_raw = sqrt(var(Braw, 0, 3));              % N x 3
    R.sig_xyz_cor = sqrt(var(Bcor, 0, 3));
    if haveBg
        [~, lb] = ismember(K, key(Sbg.P_mm), 'rows');
        Mraw = squeeze(sqrt(sum((Braw - Sbg.B_raw_G(lb, :)).^2, 2)));    % N x nRun, magnet only
        Mcor = squeeze(sqrt(sum((Bcor - Sbg.B_corr_G(lb, :)).^2, 2)));
        R.sig_mag_raw = std(Mraw, 0, 2);
        R.sig_mag_cor = std(Mcor, 0, 2);
    end
    R.sig_tilt = std(tilt, 0, 2);
    R.sig_pitch = std(pitch, 0, 2);
    R.sig_roll = std(roll, 0, 2);
end

% --- 3. dipole fits, raw and corrected ---
fitRaw = cell(nRun, 1); fitCor = cell(nRun, 1);
for k = 1:nRun
    if haveBg
        fitRaw{k} = quiet(@() fit_dipole(S{k}.file, Sbg.file, 'Plot', false));
        fitCor{k} = quiet(@() fit_dipole(S{k}.out_file, Sbg.out_file, 'Plot', false));
    else
        fitRaw{k} = quiet(@() fit_dipole(S{k}.file, '', 'Plot', false));
        fitCor{k} = quiet(@() fit_dipole(S{k}.out_file, '', 'Plot', false));
    end
end
getf = @(Fc, f) cell2mat(cellfun(@(F) F.(f), Fc, 'UniformOutput', false));
D = struct('rms_raw', getf(fitRaw, 'rms_G'), 'rms_cor', getf(fitCor, 'rms_G'), ...
           'rel_raw', getf(fitRaw, 'rel_rms'), 'rel_cor', getf(fitCor, 'rel_rms'), ...
           'r0_raw', getf(fitRaw, 'r0_mm'), 'r0_cor', getf(fitCor, 'r0_mm'), ...
           'm_raw', getf(fitRaw, 'm_abs_Am2'), 'm_cor', getf(fitCor, 'm_abs_Am2'), ...
           'fit_offset', fitRaw{1}.fit_offset);

% --- summary ---
C = struct();
C.runs = runs; C.background = ''; if haveBg, C.background = bg; end
C.reference = ref; C.P_mm = P; C.n_points = N;
C.tilt_deg = tilt; C.acc_pitch_deg = pitch; C.acc_roll_deg = roll;
C.noise_G = noisePt; C.repeat = R; C.fits = D;
C.corrected_files = cellfun(@(s) s.out_file, S, 'UniformOutput', false);
print_summary(C, haveBg);

if opt.Plot, C.figures = plot_summary(C, haveBg); else, C.figures = []; end
if opt.Save
    d = fileparts(runs{1});
    stamp = datestr(now, 'yyyymmdd_HHMMSS');
    f = fullfile(d, ['compare_tilt_runs_' stamp]);
    figs = C.figures; C = rmfield(C, 'figures');
    save([f '.mat'], 'C');
    names = {'_repeat.png', '_tilt.png', '_fits.png'};
    for i = 1:numel(figs)
        print(figs(i), [f names{i}], '-dpng', '-r150');
    end
    C.figures = figs;
    fprintf('Saved %s.mat%s\n', f, repmat(' and PNGs', 1, ~isempty(figs)));
end
end


% ---------------------------------------------------------------------------
function print_summary(C, haveBg)
nRun = numel(C.runs);
fprintf('\n=== compare_tilt_runs: %d magnet run(s)', nRun);
if haveBg, fprintf(' minus %s', short(C.background)); end
fprintf(', %d points on every run ===\n', C.n_points);
for k = 1:nRun, fprintf('  run %d: %s\n', k, short(C.runs{k})); end
fprintf('  tilt reference: centre point of %s\n', short(C.reference));
fprintf('  sensor noise of a point''s mean: median %.5f G\n', median(C.noise_G));

R = C.repeat;
if nRun >= 2
    fprintf('\nRepeatability over runs (per point)          raw          tilt-corrected\n');
    row('field vector spread, median (G)', median(R.sig_vec_raw), median(R.sig_vec_cor), '%.5f');
    row('field vector spread, 95th pct (G)', pct(R.sig_vec_raw, 95), pct(R.sig_vec_cor, 95), '%.5f');
    row('field vector spread, max (G)', max(R.sig_vec_raw), max(R.sig_vec_cor), '%.5f');
    fprintf('  %-40s [%.5f %.5f %.5f]   [%.5f %.5f %.5f]\n', 'per axis x/y/z, median (G)', ...
        median(R.sig_xyz_raw, 1), median(R.sig_xyz_cor, 1));
    if haveBg
        row('magnet field |B-Bbg| spread, median (G)', median(R.sig_mag_raw), median(R.sig_mag_cor), '%.5f');
        row('magnet field |B-Bbg| spread, max (G)', max(R.sig_mag_raw), max(R.sig_mag_cor), '%.5f');
    end
    fprintf('\nTilt repeatability over runs (per point)\n');
    fprintf('  %-40s median %.3f deg, max %.3f deg\n', 'tilt spread', median(R.sig_tilt), max(R.sig_tilt));
    fprintf('  %-40s median %.3f / %.3f deg\n', 'pitch / roll spread', median(R.sig_pitch), median(R.sig_roll));
else
    fprintf('\n(One run: no repeatability yet; needs 2 or more.)\n');
end
fprintf('  %-40s median %.3f deg, max %.3f deg\n', 'tilt relative to the reference', ...
    median(C.tilt_deg(:)), max(C.tilt_deg(:)));

D = C.fits;
fprintf('\nDipole fit%s           residual RMS (G)       rel. RMS        |m| (A m^2)     position (mm)\n', ...
    repmat(' (+ constant background)', 1, D.fit_offset));
fprintf('  %-8s %12s %10s %8s %7s %8s %8s   %-24s %s\n', 'run', 'raw', 'corrected', 'raw', 'corr', ...
    'raw', 'corr', 'raw', 'corrected');
for k = 1:numel(D.rms_raw)
    fprintf('  %-8d %12.4f %10.4f %7.1f%% %6.1f%% %8.3f %8.3f   [%6.1f %6.1f %6.1f]   [%6.1f %6.1f %6.1f]\n', ...
        k, D.rms_raw(k), D.rms_cor(k), 100 * D.rel_raw(k), 100 * D.rel_cor(k), D.m_raw(k), D.m_cor(k), ...
        D.r0_raw(k, :), D.r0_cor(k, :));
end
if numel(D.rms_raw) >= 2
    fprintf('  %-8s %12s %10s %8s %7s %8.3f %8.3f   [%6.2f %6.2f %6.2f]   [%6.2f %6.2f %6.2f]\n', ...
        'std', '', '', '', '', std(D.m_raw), std(D.m_cor), std(D.r0_raw, 0, 1), std(D.r0_cor, 0, 1));
end
fprintf('\n');
end


function row(label, a, b, fmt)
fprintf(['  %-40s ' fmt '      ' fmt '   (%+.0f %%)\n'], label, a, b, 100 * (b - a) / a);
end


function figs = plot_summary(C, haveBg)
idx = (1:C.n_points).';
nRun = numel(C.runs);
R = C.repeat;
figs = [];

if nRun >= 2
    figs(end + 1) = figure('Name', 'Tilt runs: repeatability', 'Color', 'w', 'Position', [80 80 1000 420]);
    semilogy(idx, R.sig_vec_raw, 'o-', idx, R.sig_vec_cor, 's-', idx, C.noise_G, 'k--');
    leg = {'raw', 'tilt-corrected', 'sensor noise (one point)'};
    if haveBg
        hold on; semilogy(idx, R.sig_mag_raw, '.:', idx, R.sig_mag_cor, '.-'); hold off
        leg = [leg {'magnet field |B-B_{bg}|, raw', 'magnet field |B-B_{bg}|, corrected'}];
    end
    grid on; xlabel('point'); ylabel('spread over runs (G)');
    legend(leg, 'Location', 'best');
    title(sprintf('Field spread over %d runs: median %.4f G raw, %.4f G tilt-corrected', ...
        nRun, median(R.sig_vec_raw), median(R.sig_vec_cor)));
end

figs(end + 1) = figure('Name', 'Tilt runs: tilt', 'Color', 'w', 'Position', [100 100 1000 420]);
plot(idx, C.tilt_deg, '.-');
grid on; xlabel('point'); ylabel('tilt relative to the reference (deg)');
legend(arrayfun(@(k) sprintf('run %d', k), 1:nRun, 'UniformOutput', false), 'Location', 'best');
if nRun >= 2
    title(sprintf('Tilt at each point, %d runs: spread over runs median %.3f deg', nRun, median(R.sig_tilt)));
else
    title('Tilt at each point');
end

D = C.fits;
figs(end + 1) = figure('Name', 'Tilt runs: dipole fits', 'Color', 'w', 'Position', [120 120 600 420]);
bar([D.rms_raw(:) D.rms_cor(:)]);
grid on; xlabel('run'); ylabel('dipole fit residual RMS (G)');
legend({'raw', 'tilt-corrected'}, 'Location', 'best');
title('Dipole fit residual per run');
end


% ---------------------------------------------------------------------------
function out = run_quiet(f, verbose)
% Call f() and return its result; hide what it prints unless verbose.
if verbose
    out = f();
else
    [~, out] = evalc_call(f);
end
end


function [txt, out] = evalc_call(f)
out = [];
txt = evalc('out = f();');
end


function p = pct(v, q)
v = sort(v(isfinite(v)));
if isempty(v), p = NaN; return; end
k = 1 + q / 100 * (numel(v) - 1);
p = v(floor(k)) + (k - floor(k)) * (v(ceil(k)) - v(floor(k)));
end


function n = short(f)
[~, n] = fileparts(f);
end


function files = expand_files(f, here)
if ischar(f), f = {f}; end
if isstring(f), f = cellstr(f); end
files = {};
for i = 1:numel(f)
    fi = char(f{i});
    if any(fi == '*' | fi == '?')
        d = dir(fi);
        if isempty(d), d = dir(fullfile(here, fi)); end
        if isempty(d), error('No files match %s.', fi); end
        names = sort({d.name});
        names = names(cellfun(@isempty, strfind(names, '_tiltcorr')));
        for j = 1:numel(names)
            files{end + 1} = fullfile(d(1).folder, names{j}); %#ok<AGROW>
        end
    else
        if ~isfile(fi), fi = fullfile(here, fi); end
        if ~isfile(fi), error('Cannot find %s.', f{i}); end
        files{end + 1} = fi; %#ok<AGROW>
    end
end
end


function T = read_scan(file)
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
