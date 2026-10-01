function [outFile, S] = compile_scan(logFile, opts)
% compile_scan — turn a delta-app data log (deltalog_*.csv, from File >
% Start data log) into one row per program point, in the same CSV format as
% run_field_scan, so plot_field_map works on it:
%
%   f = compile_scan("data/deltalog_20261002_141500.csv");
%   plot_field_map(f)
%
% A point is a stretch where the joint angles hold still for at least
% min_dwell_s while a program runs. The firmware reports mv = 1 during a
% program's "Wait time", so mv can't be used to find the stops; the angles
% can, because they don't change at all while the robot waits.
%
% Options (name-value):
%   settle_s     time ignored at the start of each stop (default 0.5 s)
%   min_dwell_s  shortest stop counted as a point (default 0.8 s)
%   snap_mm      round positions to this step, 0 = off (default 1 mm;
%                program points on whole mm come back exactly)
%   require_run  only count stops while a program runs (default true;
%                false also counts stops made by jogging)

arguments
    logFile (1,1) string
    opts.settle_s (1,1) double = 0.5
    opts.min_dwell_s (1,1) double = 0.8
    opts.snap_mm (1,1) double = 1
    opts.require_run (1,1) logical = true
end

T = readtable(logFile);
n = height(T);
deg = [T.deg1 T.deg2 T.deg3];

active = T.en == 1;
if opts.require_run
    active = active & T.run == 1;
end
% still(k): row k has the same angles as row k-1 (status prints 0.01 deg)
still = [false; all(abs(diff(deg)) < 0.005, 2)] & active;
runNo = cumsum([T.run(1) == 1; diff(T.run) == 1]);   % program run counter

d = diff([0; still; 0]);
segStart = max(find(d == 1) - 1, 1);   % include the row the stretch matches
segEnd   = find(d == -1) - 1;

rows = zeros(0, 13);
for k = 1:numel(segStart)
    i = (segStart(k):segEnd(k))';
    t = T.t_s(i);
    if t(end) - t(1) < opts.min_dwell_s
        continue
    end
    use = i(t >= t(1) + opts.settle_s);
    [~, first] = unique(T.b_seq(use), 'stable');   % one row per new 1044 sample
    use = use(first);
    B = [T.Bx_G(use) T.By_G(use) T.Bz_G(use)];

    xyz = median([T.x_mm(i) T.y_mm(i) T.z_mm(i)], 1, 'omitnan');
    if opts.snap_mm > 0
        xyz = round(xyz / opts.snap_mm) * opts.snap_mm;
    end
    rows(end+1, :) = [xyz, mean(B, 1, 'omitnan'), std(B, 0, 1, 'omitnan'), ...
                      max(T.e(i)), t(1), runNo(i(1)), numel(use)]; %#ok<AGROW>
end

S = array2table([(1:size(rows,1))' rows], 'VariableNames', ...
    {'idx','x_mm','y_mm','z_mm','Bx_G','By_G','Bz_G','Bx_std_G','By_std_G','Bz_std_G', ...
     'err','t_s','run_no','n_samples'});

[p, name] = fileparts(logFile);
outFile = fullfile(p, name + "_points.csv");
writetable(S, outFile);

fprintf("%s: %d log rows -> %d points", name, n, height(S));
if height(S) > 0
    fprintf(" over %d program run(s), %d-%d samples each.\n", ...
        numel(unique(S.run_no)), min(S.n_samples), max(S.n_samples));
else
    fprintf(".\n");
    warning("No stops of %.1f s or more found. Give every program point a ""Wait time"" " + ...
        "of at least 1500 ms, or pass require_run=false for jogged points.", opts.min_dwell_s);
end
fprintf("Saved %s\n", outFile);
end
