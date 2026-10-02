function S = compare_positions(groups, noiseFile, varargin)
% compare_positions — magnet repeatability at several magnet positions.
%
%   S = compare_positions("data/magP*_run*.csv", noiseFile)
%       Groups the files by the "P<k>" in their name (magP1_run1_…, magP1_run2_…,
%       magP2_run1_…), one group per magnet position.
%   S = compare_positions({"data/magP1_*.csv", "data/magP2_*.csv"}, noiseFile)
%       Or give the groups yourself, one cell per position (files, wildcard
%       patterns, or a mix). Use this to add a run with a different name, e.g. the
%       first magnet run, to position 1.
%   S = compare_positions(..., "Names", ["near +x", "centre", "near -y"])
%       Names for the positions in the table and plots (default P1, P2, ...).
%
% For each position, the runs (all on the same grid, magnet not touched between
% them) go through compare_runs: spread of |B| across runs at each point, minus
% the sensor noise floor, divided by the field gradient = position repeatability
% in mm. Moving the magnet puts the strong gradient in a different part of the
% grid, so together the positions test more of the scan volume.
%
% noiseFile is a magnet-absent scan on the same grid. It sets the noise floor
% and is subtracted to get the magnet's own field. Pass "" to skip both.
%
% Figure 1: the magnet's field on each z layer, one row per position, so you
%           can check the magnet really moved (x marks the strongest point).
% Figure 2: sigma_B and sigma_pos at every point, one column per position,
%           plus all positions pooled. Bars are medians.
%
% S.table has one row per position; S.runs{k} is compare_runs' struct for
% position k; S.pooled has the medians over all positions.

p = inputParser;
p.addParameter("Names", strings(0));
p.parse(varargin{:});
if nargin < 2, noiseFile = ""; end
haveNoise = strlength(string(noiseFile)) > 0;

% --- sort the files into positions ---
if iscell(groups)
    fileSets = cellfun(@expand_files, groups, 'UniformOutput', false);
    names = "P" + string(1:numel(fileSets));
else
    files = expand_files(groups);
    tok = strings(numel(files), 1);
    for i = 1:numel(files)
        [~, n] = fileparts(char(files(i)));
        t = regexp(n, 'P(\d+)(?=_)', 'tokens', 'once');
        if isempty(t)
            error("Can't find P<number> in %s. Name the runs magP1_run1 etc., " + ...
                  "or pass the groups as a cell array.", n);
        end
        tok(i) = t{1};
    end
    [ids, ~, g] = unique(double(tok));
    fileSets = arrayfun(@(k) files(g == k), 1:numel(ids), 'UniformOutput', false);
    names = "P" + string(ids(:).');
end
nPos = numel(fileSets);
if ~isempty(p.Results.Names)
    if numel(p.Results.Names) ~= nPos
        error("%d names given for %d positions.", numel(p.Results.Names), nPos);
    end
    names = string(p.Results.Names(:).');
end

% --- repeatability at each position ---
runs = cell(1, nPos);
for k = 1:nPos
    fprintf("\n===== %s: %d runs =====", names(k), numel(fileSets{k}));
    runs{k} = compare_runs(fileSets{k}, noiseFile, "Plot", false);
end
P = runs{1}.P;
for k = 2:nPos
    if size(runs{k}.P, 1) ~= size(P, 1) || any(abs(runs{k}.P - P) > 1e-6, 'all')
        error("%s is not on the same grid as %s. Use the same --x --y --z for every run.", ...
              names(k), names(1));
    end
end

% --- the magnet's own field (run average minus the magnet-absent scan) ---
Bmag = nan(size(P, 1), nPos);
if haveNoise
    T0 = readtable(noiseFile);
    B0 = [T0.Bx_G T0.By_G T0.Bz_G];
    for k = 1:nPos, Bmag(:, k) = vecnorm(runs{k}.Bmean - B0, 2, 2); end
    fieldWhat = "magnet field |B - B_{no magnet}|";
else
    for k = 1:nPos, Bmag(:, k) = vecnorm(runs{k}.Bmean, 2, 2); end
    fieldWhat = "|B| (no magnet-absent scan given)";
end

% --- summary table ---
nRuns = zeros(nPos, 1); nUsed = nRuns; peak = nRuns; peakAt = zeros(nPos, 3);
medB = nRuns; medPos = nRuns; p95Pos = nRuns; maxPos = nRuns;
for k = 1:nPos
    R = runs{k};
    nRuns(k)  = numel(R.files);
    nUsed(k)  = sum(isfinite(R.sigma_pos_mm));
    [peak(k), i] = max(Bmag(:, k));
    peakAt(k, :) = P(i, :);
    medB(k)   = median(R.sigma_B, 'omitnan') * 1e5;          % G -> nT
    medPos(k) = median(R.sigma_pos_mm, 'omitnan');
    p95Pos(k) = pct95(R.sigma_pos_mm);
    maxPos(k) = max(R.sigma_pos_mm);
end
tbl = table(names(:), nRuns, nUsed, peak, peakAt, medB, medPos, p95Pos, maxPos, ...
    'VariableNames', ["Position" "Runs" "PointsUsed" "PeakField_G" "PeakAt_mm" ...
                      "MedianSigmaB_nT" "MedianSigmaPos_mm" "P95SigmaPos_mm" "MaxSigmaPos_mm"]);

allPos = cell2mat(cellfun(@(R) R.sigma_pos_mm, runs, 'UniformOutput', false));
allB   = cell2mat(cellfun(@(R) R.sigma_B, runs, 'UniformOutput', false)) * 1e5;
pooled = struct('median_sigma_pos_mm', median(allPos(:), 'omitnan'), ...
                'p95_sigma_pos_mm', pct95(allPos(:)), ...
                'max_sigma_pos_mm', max(allPos(:)), ...
                'median_sigma_B_nT', median(allB(:), 'omitnan'), ...
                'points_used', sum(isfinite(allPos(:))));

fprintf("\n\n===== Summary: %d magnet positions =====\n", nPos);
disp(tbl);
fprintf("All positions pooled: sigma_pos median %.3f mm, 95th percentile %.3f mm, " + ...
        "max %.3f mm (%d point-positions).\n", pooled.median_sigma_pos_mm, ...
        pooled.p95_sigma_pos_mm, pooled.max_sigma_pos_mm, pooled.points_used);
fprintf("A peak that stays put between positions means the magnet didn't move as planned.\n");

S = struct('names', names, 'files', {fileSets}, 'table', tbl, 'runs', {runs}, ...
           'magnet_field_G', Bmag, 'pooled', pooled);

% --- figure 1: where the magnet is, one row per position ---
xs = unique(round(P(:, 1), 2)); ys = unique(round(P(:, 2), 2));
zs = sort(unique(round(P(:, 3), 2)), 'descend');
nz = numel(zs);
cl = [0 max([Bmag(:); eps])];
fig = figure('Name', "Magnet positions");
tl = tiledlayout(fig, nPos, nz, 'TileSpacing', 'compact');
for k = 1:nPos
    for j = 1:nz
        ax = nexttile(tl);
        M = nan(numel(ys), numel(xs));
        on = abs(P(:, 3) - zs(j)) < 0.01;
        [~, ix] = ismember(round(P(on, 1), 2), xs);
        [~, iy] = ismember(round(P(on, 2), 2), ys);
        M(sub2ind(size(M), iy, ix)) = Bmag(on, k);
        imagesc(ax, xs, ys, M, 'AlphaData', ~isnan(M));
        ax.CLim = cl;
        axis(ax, 'image'); ax.YDir = 'normal';
        if abs(peakAt(k, 3) - zs(j)) < 0.01
            hold(ax, 'on'); plot(ax, peakAt(k, 1), peakAt(k, 2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
            hold(ax, 'off');
        end
        if k == 1, title(ax, sprintf("z = %g mm", zs(j))); end
        if j == 1, ylabel(ax, names(k) + newline + "y (mm)"); end
        if k == nPos, xlabel(ax, 'x (mm)'); end
    end
end
cb = colorbar(ax); cb.Layout.Tile = 'east'; cb.Label.String = 'G';
title(tl, fieldWhat + ", run average");

% --- figure 2: repeatability per position ---
fig = figure('Name', "Repeatability by magnet position");
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact');
labels = [names "pooled"];
strip(nexttile(tl), [cellfun(@(R) R.sigma_B * 1e5, runs, 'UniformOutput', false) {allB(:)}], labels);
ylabel('\sigma_{|B|} across runs (nT)'); title('Field spread');
strip(nexttile(tl), [cellfun(@(R) R.sigma_pos_mm, runs, 'UniformOutput', false) {allPos(:)}], labels);
ylabel('\sigma_{pos} (mm)'); title('Position repeatability \sigma_B / |\nabla B|');
title(tl, sprintf("%d positions, pooled median %.3f mm", nPos, pooled.median_sigma_pos_mm));
end

function f = expand_files(spec)
% Files and wildcard patterns (any mix) -> a column of file names.
spec = string(spec);
f = strings(0, 1);
for s = spec(:).'
    if contains(s, ["*" "?"])
        d = dir(s);
        if isempty(d), error("No files match %s.", s); end
        f = [f; string(fullfile({d.folder}, {d.name})).'];   %#ok<AGROW>
    else
        f = [f; s];                                         %#ok<AGROW>
    end
end
end

function strip(ax, vals, labels)
% One column of dots per group (jittered sideways) with a bar at the median.
hold(ax, 'on');
for k = 1:numel(vals)
    v = vals{k}(isfinite(vals{k}));
    jit = 0.15 * (rand(size(v)) - 0.5) * 2;
    plot(ax, k + jit, v, 'o', 'MarkerSize', 4);
    if ~isempty(v)
        plot(ax, k + [-0.3 0.3], median(v) * [1 1], 'k-', 'LineWidth', 2);
    end
end
hold(ax, 'off'); grid(ax, 'on');
xlim(ax, [0.5 numel(vals) + 0.5]);
xticks(ax, 1:numel(vals)); xticklabels(ax, labels);
end

function p = pct95(v)
% 95th percentile without the Statistics Toolbox (linear interpolation).
v = sort(v(isfinite(v)));
if isempty(v), p = NaN; return; end
k = 1 + 0.95 * (numel(v) - 1);
p = v(floor(k)) + (k - floor(k)) * (v(ceil(k)) - v(floor(k)));
end
