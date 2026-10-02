function S = compare_repeatability(RA, RB, varargin)
% compare_repeatability — two compare_runs results side by side, e.g. correction off
% vs hybrid, on the same target grid.
%
%   Roff = compare_runs([off1 off2], noiseFile);
%   Rhyb = compare_runs([hyb1 hyb2], noiseFile);
%   S = compare_repeatability(Roff, Rhyb, 'Names', {'off', 'hybrid'})
%   S = compare_repeatability(Roff, Rhyb, 'Names', {'off', 'hybrid'}, 'Save', 'data/off_vs_hybrid')
%
% Figure 1: histograms of sigma_pos (mm) for both, on the same bins.
% Figure 2: point by point, B against A, for sigma_B (nT) and sigma_pos (mm).
%           Below the dashed y = x line, B is more repeatable at that point.
% S is a struct of the summary numbers, also printed.
%
% sigma_pos depends on each set's own field gradient, which is not the same if the
% probe ended up in different places (as it does with the correction on), so compare
% sigma_B as well. With only 2 runs per set every sigma is a rough estimate.

p = inputParser;
p.addParameter("Names", {'A', 'B'});   % cell array, or a string array in MATLAB
p.addParameter("Save", '');
p.parse(varargin{:});
opt = p.Results;
names = cellstr(opt.Names);
if numel(names) ~= 2, error("'Names' needs two names."); end
need = {'P', 'sigma_B', 'sigma_pos_mm', 'files'};
if ~isstruct(RA) || ~isstruct(RB) || ~isscalar(RA) || ~isscalar(RB) || ~all(isfield(RA, need)) || ~all(isfield(RB, need))
    error("Pass two structs returned by compare_runs.");
end
if size(RA.P, 1) ~= size(RB.P, 1) || any(abs(RA.P(:) - RB.P(:)) > 1e-6)
    error("The two sets are not on the same target grid.");
end

sA = RA.sigma_pos_mm; sB = RB.sigma_pos_mm;
bA = RA.sigma_B * 1e5; bB = RB.sigma_B * 1e5;       % G -> nT, as in compare_runs
both = isfinite(sA) & isfinite(sB);

S = struct();
S.names = names;
S.runs = [numel(RA.files) numel(RB.files)];
S.sigma_pos_median_mm = [median(sA(isfinite(sA))) median(sB(isfinite(sB)))];
S.sigma_pos_p95_mm = [pct95(sA) pct95(sB)];
S.sigma_pos_usable = [nnz(isfinite(sA)) nnz(isfinite(sB))];
S.sigma_B_median_nT = [median(bA(isfinite(bA))) median(bB(isfinite(bB)))];
S.points_compared = nnz(both);
S.points_B_better = nnz(sB(both) < sA(both));

fprintf("\n%-22s %12s %12s\n", "", names{1}, names{2});
fprintf("%-22s %12d %12d\n", "runs", S.runs);
fprintf("%-22s %12.3f %12.3f\n", "sigma_pos median (mm)", S.sigma_pos_median_mm);
fprintf("%-22s %12.3f %12.3f\n", "sigma_pos 95th (mm)", S.sigma_pos_p95_mm);
fprintf("%-22s %12d %12d\n", "points usable", S.sigma_pos_usable);
fprintf("%-22s %12.0f %12.0f\n", "sigma_B median (nT)", S.sigma_B_median_nT);
fprintf("%s has the smaller sigma_pos at %d of %d points usable in both.\n", ...
    names{2}, S.points_B_better, S.points_compared);

% --- 1: histograms on the same bins ---
allv = [sA(isfinite(sA)); sB(isfinite(sB))];
if isempty(allv), error("No usable sigma_pos in either set."); end
edges = linspace(0, max(allv) * 1.0001, 13);
nA = histc(sA(isfinite(sA)), edges); nB = histc(sB(isfinite(sB)), edges);   % histc: MATLAB and Octave
nA = nA(:); nB = nB(:);
nA(end - 1) = nA(end - 1) + nA(end); nB(end - 1) = nB(end - 1) + nB(end);   % last edge into last bin
ctr = (edges(1:end - 1) + edges(2:end)) / 2;
f1 = figure("Name", ['sigma_pos: ' names{1} ' vs ' names{2}], "Color", "w", "Position", [80 80 760 480]);
bar(ctr, [nA(1:end - 1) nB(1:end - 1)], 1, "grouped");
grid on
xlabel('\sigma_{pos} (mm)'); ylabel("points");
legend(sprintf("%s (median %.2f mm)", names{1}, S.sigma_pos_median_mm(1)), ...
       sprintf("%s (median %.2f mm)", names{2}, S.sigma_pos_median_mm(2)), "Location", "northeast");
title(sprintf("Position repeatability, %s vs %s", names{1}, names{2}));

% --- 2: point by point ---
f2 = figure("Name", ['Point by point: ' names{1} ' vs ' names{2}], "Color", "w", ...
    "Position", [80 80 1000 460]);
subplot(1, 2, 1);
okb = isfinite(bA) & isfinite(bB);
scatter_xy(bA(okb), bB(okb));
xlabel(sprintf('\\sigma_B, %s (nT)', names{1})); ylabel(sprintf('\\sigma_B, %s (nT)', names{2}));
title("Spread of |B| across runs");
subplot(1, 2, 2);
scatter_xy(sA(both), sB(both));
xlabel(sprintf('\\sigma_{pos}, %s (mm)', names{1})); ylabel(sprintf('\\sigma_{pos}, %s (mm)', names{2}));
title(sprintf("Position repeatability (%d points usable in both)", S.points_compared));

if ~isempty(char(opt.Save))
    pre = char(opt.Save);
    print(f1, [pre '_hist.png'], '-dpng', '-r150');
    print(f2, [pre '_points.png'], '-dpng', '-r150');
    fprintf("Saved %s_hist.png and %s_points.png\n", pre, pre);
end
end


function scatter_xy(a, b)
plot(a, b, "o", "MarkerFaceColor", [0 0.447 0.741], "MarkerEdgeColor", "none"); hold on
m = max([a; b; eps]) * 1.05;
plot([0 m], [0 m], "k--");
hold off
axis equal; grid on
xlim([0 m]); ylim([0 m]);
end


function p = pct95(v)
% 95th percentile without the Statistics Toolbox (linear interpolation), as in compare_runs.
v = sort(v(isfinite(v)));
if isempty(v), p = NaN; return; end
k = 1 + 0.95 * (numel(v) - 1);
p = v(floor(k)) + (k - floor(k)) * (v(ceil(k)) - v(floor(k)));
end
