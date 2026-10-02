function plot_repeatability_layers(R, varargin)
% plot_repeatability_layers — compare_runs results as maps, one panel per z layer.
%
%   R = compare_runs(files, noiseFile);
%   plot_repeatability_layers(R)
%   plot_repeatability_layers(R, 'Title', "correction off")   % text added to the figure titles
%   plot_repeatability_layers(R, 'Save', "data/repeat_off")   % also save <prefix>_sigma_pos.png
%                                                             % and <prefix>_gradient.png
%
% Figure 1: sigma_pos (mm), the position repeatability at each grid point.
%           Grey with an x = gradient too weak for sigma_pos to mean anything.
% Figure 2: |grad |B|| (nT/mm), the field gradient sigma_pos is divided by. Where it
%           is large the magnet gives a strong position signal; where it is small,
%           sigma_pos is unreliable or flagged.
% Both use one colour scale for all layers.

p = inputParser;
p.addParameter("Title", '');
p.addParameter("Save", '');
p.parse(varargin{:});
opt = p.Results;
if ~isstruct(R) || ~isscalar(R) || ~all(isfield(R, {'P', 'sigma_pos_mm', 'grad', 'weak_gradient'}))
    error("Pass the struct returned by compare_runs.");
end

P = R.P;
xs = unique(round(100 * P(:, 1)) / 100); ys = unique(round(100 * P(:, 2)) / 100);
zs = sort(unique(round(100 * P(:, 3)) / 100), "descend");
nz = numel(zs);
if numel(xs) < 2 || numel(ys) < 2
    error("Need at least 2 points along x and y for a map.");
end
dx = min(diff(xs)); dy = min(diff(ys));
pad = [-0.5 0.5];
cols = min(nz, 3); rows = ceil(nz / cols);
figsize = [80 80 520 * cols 470 * rows];
extra = '';
if ~isempty(char(opt.Title)), extra = [' - ' char(opt.Title)]; end
n = numel(R.files);

f1 = layer_maps(P, R.sigma_pos_mm, R.weak_gradient, xs, ys, zs, rows, cols, dx, dy, pad, figsize, ...
    "%.2f", '\sigma_{pos} (mm)', ...
    sprintf("Position repeatability sigma_pos = sigma_B / |grad B|, %d runs%s   (x = gradient too weak)", ...
    n, extra));
f2 = layer_maps(P, R.grad * 1e5, false(size(R.grad)), xs, ys, zs, rows, cols, dx, dy, pad, figsize, ...
    "%.0f", '|\nabla|B|| (nT/mm)', ...
    sprintf("Field gradient |grad |B|| of the run-averaged field, %d runs%s", n, extra));

if ~isempty(char(opt.Save))
    pre = char(opt.Save);
    print(f1, [pre '_sigma_pos.png'], '-dpng', '-r150');
    print(f2, [pre '_gradient.png'], '-dpng', '-r150');
    fprintf("Saved %s_sigma_pos.png and %s_gradient.png\n", pre, pre);
end
end


function f = layer_maps(P, v, flag, xs, ys, zs, rows, cols, dx, dy, pad, figsize, fmt, cblab, ttl)
good = isfinite(v) & ~flag;
lim = [min(v(good)) max(v(good))];
if isempty(lim), lim = [0 1]; end
if diff(lim) == 0, lim = lim + [-1 1] * max(abs(lim(1)) * 0.01, 1e-6); end
f = figure("Name", ttl, "Color", "w", "Position", figsize);
[X, Y] = meshgrid(xs, ys);
for k = 1:numel(zs)
    subplot(rows, cols, k);
    M = to_grid(P, v, xs, ys, zs(k));
    Fl = to_grid(P, double(flag | ~isfinite(v)), xs, ys, zs(k)) > 0;   % flagged or missing
    M(Fl) = NaN;
    imagesc(xs, ys, M, "AlphaData", double(~isnan(M))); hold on
    set(gca, "YDir", "normal", "Color", [0.85 0.85 0.85]);              % grey shows through NaN
    plot(X(Fl), Y(Fl), "kx", "MarkerSize", 8, "LineWidth", 1.2);
    for i = find(~isnan(M(:)))'
        tc = [0.1 0.1 0.1];
        if (M(i) - lim(1)) / diff(lim) < 0.4, tc = [1 1 1]; end   % white on the dark end of the scale
        text(X(i), Y(i), sprintf(fmt, M(i)), "FontSize", 7, ...
            "HorizontalAlignment", "center", "Color", tc);
    end
    hold off
    caxis(lim); colormap(gca, seq_map());
    axis equal tight
    set(gca, "XTick", xs, "YTick", ys);
    xlim([xs(1) xs(end)] + pad * dx); ylim([ys(1) ys(end)] + pad * dy);
    xlabel("x (mm)"); ylabel("y (mm)");
    title(sprintf("z = %.0f mm", zs(k)));
    cb = colorbar; ylabel(cb, cblab);
end
suptitle_compat(f, ttl);
end


function M = to_grid(P, v, xs, ys, z)
M = nan(numel(ys), numel(xs));
on = abs(P(:, 3) - z) < 0.05 & isfinite(v);
[~, ix] = min(abs(P(on, 1) - xs.'), [], 2);
[~, iy] = min(abs(P(on, 2) - ys.'), [], 2);
M(sub2ind(size(M), iy, ix)) = v(on);
end


function cmap = seq_map()
% parula in MATLAB, viridis in Octave (no parula there)
if exist("parula", "file") || exist("parula", "builtin")
    cmap = parula(256);
else
    cmap = viridis(256);
end
end


function suptitle_compat(fig, s)
figure(fig);
if exist("sgtitle", "file") || exist("sgtitle", "builtin")
    sgtitle(s, "FontSize", 12, "Interpreter", "none");
else
    annotation("textbox", [0 0.94 1 0.05], "String", s, "EdgeColor", "none", ...
        "HorizontalAlignment", "center", "FontSize", 12, "Interpreter", "none");
end
end
