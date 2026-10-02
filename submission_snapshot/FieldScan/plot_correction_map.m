function plot_correction_map(dataFile, varargin)
% plot_correction_map — what the position correction did: where each point was aimed
% (sent_x_mm..sent_z_mm) compared with its target (x_mm..z_mm), one panel per z layer.
%
%   plot_correction_map(file)                  % a scan run with --correction hybrid
%   plot_correction_map(file, 'Save', true)    % also save <name>_correction.png next to the CSV
%
% Arrows = horizontal offset (sent - target), drawn to scale in mm on the scan grid.
% Colour = vertical offset (sent z - target z), mm.
% Needs the sent_* columns, which field_scan.py writes. With the correction off,
% sent = target everywhere and the function says so.

p = inputParser;
p.addParameter("Save", false);
p.parse(varargin{:});
opt = p.Results;
dataFile = resolve_path(dataFile);

T = read_scan(dataFile);
if ~all(isfield(T, {'sent_x_mm', 'sent_y_mm', 'sent_z_mm'}))
    error("%s has no sent_x_mm..sent_z_mm columns (scan from run_field_scan.m or an older field_scan.py).", ...
        dataFile);
end
P = [T.x_mm T.y_mm T.z_mm];
D = [T.sent_x_mm T.sent_y_mm T.sent_z_mm] - P;     % offset, mm
ok = all(isfinite(D), 2);
P = P(ok, :); D = D(ok, :);
if isempty(P), error("No points with a sent position in %s.", dataFile); end
[~, name] = fileparts(char(dataFile));
dist = sqrt(sum(D.^2, 2));
if max(dist) < 1e-6
    warning("%s: sent = target at every point, so the correction was off. Nothing to map.", name);
    return
end

xs = unique(P(:, 1)); ys = unique(P(:, 2)); zs = sort(unique(P(:, 3)), "descend");
nz = numel(zs);
if numel(xs) < 2 || numel(ys) < 2
    error("Need at least 2 points along x and y for a map.");
end
layer = @(v, z) to_grid(P, v, xs, ys, z);
dx = min(diff(xs)); dy = min(diff(ys));
pad = [-0.5 0.5];
cols = min(nz, 3); rows = ceil(nz / cols);

[lim, cmap] = sign_scale(D(:, 3));
f = figure("Name", ['Correction: ' name], "Color", "w", "Position", [80 80 520 * cols 470 * rows]);
for k = 1:nz
    subplot(rows, cols, k);
    Dz = layer(D(:, 3), zs(k));
    imagesc(xs, ys, Dz, "AlphaData", double(~isnan(Dz))); hold on
    set(gca, "YDir", "normal");
    [X, Y] = meshgrid(xs, ys);
    U = layer(D(:, 1), zs(k)); V = layer(D(:, 2), zs(k));
    plot(X(:), Y(:), "k.", "MarkerSize", 10);                       % targets
    quiver(X, Y, U, V, 0, "k", "LineWidth", 1.2, "MaxHeadSize", 0.5);   % to scale, mm
    for i = find(~isnan(U(:)))'
        sv = -sign(V(i)); if sv == 0, sv = -1; end              % label on the side away from the arrow
        text(X(i), Y(i) + sv * 0.25 * dy, sprintf("%.1f", hypot(U(i), V(i))), "FontSize", 7, ...
            "HorizontalAlignment", "center", "Color", [0.1 0.1 0.1]);
    end
    hold off
    caxis(lim); colormap(gca, cmap);
    axis equal tight
    set(gca, "XTick", xs, "YTick", ys);
    xlim([xs(1) xs(end)] + pad * dx); ylim([ys(1) ys(end)] + pad * dy);
    xlabel("x (mm)"); ylabel("y (mm)");
    on = abs(P(:, 3) - zs(k)) < 0.05;
    title(sprintf("z = %.0f mm   (max %.1f mm)", zs(k), max(hypot(D(on, 1), D(on, 2)))));
    cb = colorbar; ylabel(cb, "sent z - target z (mm)");
end
suptitle_compat(f, sprintf(['Position correction: arrows = sent - target in x-y (to scale, mm, ' ...
    'value printed), colour = z offset   (%s)'], name));

fprintf("%s: %d points, offset |sent - target| median %.2f mm, max %.2f mm; z offset %.2f to %.2f mm\n", ...
    name, numel(dist), median(dist), max(dist), min(D(:, 3)), max(D(:, 3)));

if opt.Save
    [d, n] = fileparts(char(dataFile));
    print(f, fullfile(d, [n '_correction.png']), '-dpng', '-r150');
    fprintf("Saved %s_correction.png in %s\n", n, d);
end
end


function M = to_grid(P, v, xs, ys, z)
M = nan(numel(ys), numel(xs));
on = abs(P(:, 3) - z) < 0.05 & isfinite(v);
[~, ix] = min(abs(P(on, 1) - xs.'), [], 2);
[~, iy] = min(abs(P(on, 2) - ys.'), [], 2);
M(sub2ind(size(M), iy, ix)) = v(on);
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


function [cl, cmap] = sign_scale(v)
% Colour scale for a signed value. One sign everywhere: white -> red (+) or blue (-)
% over the data range. Both signs: blue -> white -> red, centred on 0.
v = v(isfinite(v));
m = redblue(256);
if all(v >= 0)
    cl = [0 max(v)]; cmap = m(129:end, :);
elseif all(v <= 0)
    cl = [min(v) 0]; cmap = m(1:128, :);
else
    cl = [-1 1] * max(abs(v)); cmap = m;
end
if diff(cl) == 0, cl = cl + [-1 1] * 1e-3; end
end


function cmap = redblue(n)
% blue (negative) -> white -> red (positive)
t = linspace(-1, 1, n).';
cmap = [min(1, 1 + t) min(1, 1 - abs(t)) min(1, 1 - t)];
cmap = 0.15 + 0.85 * cmap;
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


function f = resolve_path(f)
% Accept a path relative to the current folder or to this script's folder
% (FieldScan), so "data/....csv" works from anywhere.
f = char(f);
if isempty(f) || isfile(f), return; end
here = fileparts(mfilename("fullpath"));
alt = fullfile(here, f);
if isfile(alt), f = alt; return; end
error("Cannot find %s\nLooked in %s and in %s.\nList the scans with:  dir(fullfile('%s', 'data'))", ...
    f, pwd, here, here);
end
