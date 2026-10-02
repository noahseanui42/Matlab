function plot_field_layers(dataFile, baselineFile, varargin)
% plot_field_layers — heat map and direction map of a scan, one panel per z layer,
% side views (x-z or y-z), and a 3D map with the layers stacked.
%
%   plot_field_layers(file)                  % raw field (includes Earth's field)
%   plot_field_layers(file, baselineFile)    % file minus a reference scan, matched
%                                            % point by point on position (e.g. magnet
%                                            % run minus a no-magnet run = magnet only)
%   plot_field_layers(file, '', 'Save', true)        % also save PNGs next to the CSV
%   plot_field_layers(file, '', 'Labels', false)     % no values printed on the heat map
%   plot_field_layers(file, '', 'Side', 'yz')        % side views in y-z instead of x-z
%   plot_field_layers(file, '', 'R', R)              % sensor -> robot rotation
%                                                    % (default: scan_config's R_sensor_to_robot)
%
% Works on CSVs from field_scan.py and run_field_scan.m. Positions are the
% TARGET grid points (x_mm..z_mm), which is where each reading belongs.
%
% Figure 1, heat map:      |B| in gauss on each z layer, interpolated between the
%                          grid points (dots), one colour scale for all layers.
% Figure 2, direction map: arrows = direction of the field in the x-y plane (unit
%                          length, so weak and strong fields read the same);
%                          colour = vertical component Bz (red = +z, blue = -z;
%                          white = smallest |Bz| when Bz has one sign throughout).
% Figure 3, side views:    the field seen from the side, one panel per y row (x-z
%                          plane; "Side", "yz" for y-z panels, one per x column).
%                          Arrows = field direction in that plane (unit length);
%                          colour = the component out of the page (By for x-z).
% Figure 4, 3D map:        the layers stacked at their real heights (|B| colour,
%                          same scale as figure 1) with 3D arrows showing the full
%                          field direction at every point. Rotate it with the mouse.

p = inputParser;
p.addParameter("Save", false);
p.addParameter("Labels", true);
p.addParameter("R", []);
p.addParameter("Side", "xz");      % side views: "xz" (one panel per y) or "yz" (one per x)
p.parse(varargin{:});
opt = p.Results;
if nargin < 2, baselineFile = ''; end

R = opt.R;
if isempty(R)
    R = eye(3);
    if exist("scan_config", "file")
        cfg = scan_config();
        if isfield(cfg, "R_sensor_to_robot"), R = cfg.R_sensor_to_robot; end
    end
end

T = read_scan(dataFile);
P = [T.x_mm T.y_mm T.z_mm];
B = [T.Bx_G T.By_G T.Bz_G];
what = 'Raw field';
if ~isempty(baselineFile)
    T0 = read_scan(baselineFile);
    [found, loc] = ismember(round(10 * P), round(10 * [T0.x_mm T0.y_mm T0.z_mm]), "rows");   % to 0.1 mm
    if ~any(found)
        error("Baseline has no points in common with the data file.");
    elseif ~all(found)
        warning("%d of %d points have no baseline at the same position and are left out.", ...
            nnz(~found), numel(found));
    end
    B0 = nan(size(B));
    B0(found, :) = [T0.Bx_G(loc(found)) T0.By_G(loc(found)) T0.Bz_G(loc(found))];
    B = B - B0;
    [~, n0] = fileparts(char(baselineFile));
    what = ['Field minus ' n0];
end
B = (R * B.').';                     % sensor axes -> robot axes
Bmag = sqrt(sum(B.^2, 2));

xs = unique(P(:, 1)); ys = unique(P(:, 2)); zs = sort(unique(P(:, 3)), "descend");
nz = numel(zs);
if numel(xs) < 2 || numel(ys) < 2
    error("Need at least 2 points along x and y for a map.");
end
layer = @(v, z) to_grid(P, v, xs, ys, z);   % (ny x nx) matrix for layer z, NaN where missing

[~, name] = fileparts(char(dataFile));
dx = min(diff(xs)); dy = min(diff(ys));
pad = [-0.5 0.5];
cols = min(nz, 3); rows = ceil(nz / cols);

% --- 1: heat map of |B| ---
figsize = [80 80 520 * cols 470 * rows];
f1 = figure("Name", ['Heat map: ' name], "Color", "w", "Position", figsize);
lim = [min(Bmag) max(Bmag)];
if diff(lim) == 0, lim = lim + [-1 1] * 1e-3; end
for k = 1:nz
    subplot(rows, cols, k);
    M = layer(Bmag, zs(k));
    [xf, yf] = meshgrid(linspace(xs(1), xs(end), 101), linspace(ys(1), ys(end), 101));
    Mf = interp2(xs, ys, M, xf, yf, "linear");
    imagesc(xf(1, :), yf(:, 1), Mf, "AlphaData", double(~isnan(Mf))); hold on
    set(gca, "YDir", "normal");
    [X, Y] = meshgrid(xs, ys);
    plot(X(:), Y(:), "k.", "MarkerSize", 8);
    if opt.Labels
        for i = find(~isnan(M(:)))'
            text(X(i), Y(i) + 0.22 * dy, sprintf("%.3f", M(i)), "FontSize", 7, ...
                "HorizontalAlignment", "center", "Color", [0.1 0.1 0.1]);
        end
    end
    hold off
    caxis(lim); colormap(gca, seq_map());
    axis equal tight
    set(gca, "XTick", xs, "YTick", ys);
    xlim([xs(1) xs(end)] + pad * dx); ylim([ys(1) ys(end)] + pad * dy);
    xlabel("x (mm)"); ylabel("y (mm)");
    title(sprintf("z = %.0f mm", zs(k)));
    cb = colorbar; ylabel(cb, "|B| (G)");
end
suptitle_compat(f1, sprintf("%s: |B|   (%s)", what, name));

% --- 2: direction map ---
f2 = figure("Name", ['Direction map: ' name], "Color", "w", "Position", figsize);
for k = 1:nz
    subplot(rows, cols, k);
    Bx = layer(B(:, 1), zs(k)); By = layer(B(:, 2), zs(k)); Bz = layer(B(:, 3), zs(k));
    imagesc(xs, ys, Bz, "AlphaData", double(~isnan(Bz))); hold on
    set(gca, "YDir", "normal");
    h = hypot(Bx, By);
    L = 0.8 * min(dx, dy);                 % arrow length, mm
    U = L * Bx ./ h; V = L * By ./ h;
    U(h == 0) = 0; V(h == 0) = 0;
    [X, Y] = meshgrid(xs, ys);
    quiver(X - U / 2, Y - V / 2, U, V, 0, "k", "LineWidth", 1, "MaxHeadSize", 0.4);
    hold off
    [cl, cm] = sign_scale(B(:, 3));
    caxis(cl); colormap(gca, cm);
    axis equal tight
    set(gca, "XTick", xs, "YTick", ys);
    xlim([xs(1) xs(end)] + pad * dx); ylim([ys(1) ys(end)] + pad * dy);
    xlabel("x (mm)"); ylabel("y (mm)");
    title(sprintf("z = %.0f mm", zs(k)));
    cb = colorbar; ylabel(cb, "B_z (G)");
end
suptitle_compat(f2, sprintf("%s: arrows = field direction in the x-y plane, colour = Bz   (%s)", ...
    what, name));

% --- 3: side views (x-z panels, one per y; or y-z panels, one per x) ---
if strcmpi(opt.Side, "yz")
    ia = 2; ib = 1; as = ys; bs = xs; an = "y"; bn = "x"; oc = 1;   % horizontal y, panels per x, out of page Bx
else
    ia = 1; ib = 2; as = xs; bs = ys; an = "x"; bn = "y"; oc = 2;   % horizontal x, panels per y, out of page By
end
zsa = sort(zs);
nb = numel(bs); scols = min(nb, 5); srows = ceil(nb / scols);
f3 = figure("Name", ['Side views: ' name], "Color", "w", ...
    "Position", [80 80 max(380 * scols, 700) 330 * srows + 80]);
da = min(diff(as)); dzs = da;
if numel(zsa) > 1, dzs = min(diff(zsa)); end
L = 0.8 * min(da, dzs);
[cl, cm] = sign_scale(B(:, oc));
for k = 1:nb
    subplot(srows, scols, k);
    on = abs(P(:, ib) - bs(k)) < 0.05;
    Ma = side_grid(P(on, ia), P(on, 3), B(on, ia), as, zsa);
    Mz = side_grid(P(on, ia), P(on, 3), B(on, 3), as, zsa);
    Mo = side_grid(P(on, ia), P(on, 3), B(on, oc), as, zsa);
    imagesc(as, zsa, Mo, "AlphaData", double(~isnan(Mo))); hold on
    set(gca, "YDir", "normal");
    h = hypot(Ma, Mz);
    U = L * Ma ./ h; W = L * Mz ./ h;
    [A, Zg] = meshgrid(as, zsa);
    quiver(A - U / 2, Zg - W / 2, U, W, 0, "k", "LineWidth", 1, "MaxHeadSize", 0.4);
    hold off
    caxis(cl); colormap(gca, cm);
    axis equal tight
    set(gca, "XTick", as, "YTick", zsa);
    xlim([as(1) as(end)] + pad * da); ylim([zsa(1) zsa(end)] + pad * dzs);
    xlabel(sprintf('%s (mm)', an)); ylabel('z (mm)');
    title(sprintf("%s = %.0f mm", bn, bs(k)));
    if mod(k, scols) == 0 || k == nb
        cb = colorbar; ylabel(cb, sprintf('B%s (G)', bn));
    end
end
suptitle_compat(f3, sprintf("%s: side view, arrows = field direction in the %s-z plane, colour = B%s   (%s)", ...
    what, an, bn, name));

% --- 4: 3D map, layers stacked at their real heights ---
f4 = figure("Name", ['3D map: ' name], "Color", "w", "Position", [80 80 900 750]);
[xf, yf] = meshgrid(linspace(xs(1), xs(end), 101), linspace(ys(1), ys(end), 101));
[X, Y] = meshgrid(xs, ys);
L = 0.6 * min(dx, dy);                     % arrow length, mm
for k = 1:nz
    Mf = interp2(xs, ys, layer(Bmag, zs(k)), xf, yf, "linear");
    surf(xf, yf, zs(k) * ones(size(xf)), Mf, "EdgeColor", "none", "FaceAlpha", 0.8); hold on
    Bx = layer(B(:, 1), zs(k)); By = layer(B(:, 2), zs(k)); Bz = layer(B(:, 3), zs(k));
    h = sqrt(Bx.^2 + By.^2 + Bz.^2);
    U = L * Bx ./ h; V = L * By ./ h; W = L * Bz ./ h;
    Z = zs(k) * ones(size(X));
    quiver3(X - U / 2, Y - V / 2, Z - W / 2, U, V, W, 0, "k", "LineWidth", 1, "MaxHeadSize", 0.4);
end
hold off
caxis(lim); colormap(gca, seq_map());
cb = colorbar; ylabel(cb, "|B| (G)");
axis equal; grid on; box on
set(gca, "XTick", xs, "YTick", ys, "ZTick", sort(zs));
xlim([xs(1) xs(end)] + pad * dx); ylim([ys(1) ys(end)] + pad * dy);
xlabel("x (mm)"); ylabel("y (mm)"); zlabel("z (mm)");
view(-35, 25);
title(sprintf("%s: |B| layers, arrows = field direction   (%s)", what, name), "Interpreter", "none");

if opt.Save
    [d, n] = fileparts(char(dataFile));
    print(f1, fullfile(d, [n '_heatmap.png']), '-dpng', '-r150');
    print(f2, fullfile(d, [n '_direction.png']), '-dpng', '-r150');
    print(f3, fullfile(d, [n '_side.png']), '-dpng', '-r150');
    print(f4, fullfile(d, [n '_3d.png']), '-dpng', '-r150');
    fprintf("Saved %s_heatmap.png, _direction.png, _side.png and _3d.png in %s\n", n, d);
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


function cmap = seq_map()
% parula in MATLAB, viridis in Octave (no parula there)
if exist("parula", "file") || exist("parula", "builtin")
    cmap = parula(256);
else
    cmap = viridis(256);
end
end


function M = side_grid(a, z, v, as, zs)
% (numel(zs) x numel(as)) matrix for one side-view panel, NaN where missing
M = nan(numel(zs), numel(as));
ok = isfinite(v);
[~, ia] = min(abs(a(ok) - as.'), [], 2);
[~, iz] = min(abs(z(ok) - zs.'), [], 2);
M(sub2ind(size(M), iz, ia)) = v(ok);
end


function [cl, cmap] = sign_scale(bz)
% Colour scale for a signed component. One sign everywhere (e.g. raw Bz, Earth's
% field): white -> red (+) or blue (-) over the data range, so small changes show.
% Both signs (e.g. a baseline-subtracted magnet field): blue -> white -> red,
% centred on 0.
bz = bz(isfinite(bz));
m = redblue(256);
if all(bz >= 0)
    cl = [min(bz) max(bz)]; cmap = m(129:end, :);
elseif all(bz <= 0)
    cl = [min(bz) max(bz)]; cmap = m(1:128, :);
else
    cl = [-1 1] * max(abs(bz)); cmap = m;
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
