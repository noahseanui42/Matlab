function S = plot_tilt(dataFile, varargin)
% plot_tilt — tilt map of a field_scan_tilt.py scan, plus the settling record.
%
%   plot_tilt(file)                          % tilt relative to the grid centre
%   plot_tilt(file, 'Reference', otherFile)  % same reference options as tilt_correct
%   plot_tilt(file, 'Save', true)            % also save PNGs next to the CSV
%   plot_tilt(file, 'R', R)                  % sensor -> robot rotation for the arrows
%                                            % (default: scan_config's R_sensor_to_robot)
%
% Figure 1, one panel per z layer: colour = tilt (deg) relative to the reference;
%   arrows = horizontal shift of the measured gravity direction, in robot x-y. If the
%   accelerometer reports gravity pointing down, the arrows point downhill (to the
%   side that dips). Check the sign once by tipping the board by hand.
% Figure 2: tilt vs horizontal distance from the robot axis (colour = z), the gyro's
%   angular rate while sampling, and the settle time at each point.
% Returns tilt_correct's struct.

p = inputParser;
p.addParameter('Reference', 'centre');
p.addParameter('R', []);
p.addParameter('Save', false);
p.parse(varargin{:});
opt = p.Results;

S = tilt_correct(dataFile, 'Reference', opt.Reference);
T = read_scan(S.file);
R = opt.R;
if isempty(R)
    R = eye(3);
    if exist('scan_config', 'file')
        cfg = scan_config();
        if isfield(cfg, 'R_sensor_to_robot'), R = cfg.R_sensor_to_robot; end
    end
end

ok = S.ok;
P = S.P_mm;
tilt = S.tilt_deg;
d = (R * (S.g - S.g_ref).').';                   % shift of gravity, robot axes
xs = unique(P(ok, 1)); ys = unique(P(ok, 2)); zs = sort(unique(P(ok, 3)), 'descend');
nz = numel(zs);
[~, name] = fileparts(S.file);

% --- 1: tilt map per layer ---
cols = min(nz, 3); rows = ceil(nz / cols);
f1 = figure('Name', ['Tilt map: ' name], 'Color', 'w', 'Position', [80 80 480 * cols 430 * rows]);
lim = [0 max([tilt(ok); 1e-3])];
dx = step(xs); dy = step(ys);
for k = 1:nz
    subplot(rows, cols, k);
    M = to_grid(P, tilt, xs, ys, zs(k), ok);
    U = to_grid(P, d(:, 1), xs, ys, zs(k), ok);
    V = to_grid(P, d(:, 2), xs, ys, zs(k), ok);
    imagesc(xs, ys, M); hold on
    set(gca, 'YDir', 'normal');
    h = hypot(U, V);
    L = 0.8 * min(dx, dy);
    U = L * U ./ max(h, eps); V = L * V ./ max(h, eps);
    [X, Y] = meshgrid(xs, ys);
    quiver(X - U / 2, Y - V / 2, U, V, 0, 'k', 'LineWidth', 1);
    hold off
    caxis(lim); colorbar;
    axis equal tight
    xlabel('x (mm)'); ylabel('y (mm)');
    title(sprintf('z = %.0f mm, tilt (deg)', zs(k)));
end

% --- 2: tilt vs radius, gyro and settling ---
f2 = figure('Name', ['Tilt and settling: ' name], 'Color', 'w', 'Position', [100 100 1200 380]);
subplot(1, 3, 1);
r = hypot(P(:, 1), P(:, 2));
scatter(r(ok), tilt(ok), 30, P(ok, 3), 'filled');
cb = colorbar; ylabel(cb, 'z (mm)');
grid on; xlabel('distance from the robot axis (mm)'); ylabel('tilt (deg)');
title('Tilt vs distance off-axis');
idx = (1:numel(tilt)).';
subplot(1, 3, 2);
if isfield(T, 'gyro_rms_dps')
    plot(idx, T.gyro_rms_dps, 'o-', idx, T.gyro_max_dps, '.');
    legend({'RMS', 'max'}, 'Location', 'best');
end
grid on; xlabel('point'); ylabel('angular rate while sampling (deg/s)');
title('Gyro: was the probe still?');
subplot(1, 3, 3);
if isfield(T, 'settle_s')
    plot(idx, T.settle_s, 'o-'); hold on
    late = T.settled == 0;
    if any(late), plot(idx(late), T.settle_s(late), 'rx', 'MarkerSize', 10, 'LineWidth', 2); end
    hold off
end
grid on; xlabel('point'); ylabel('settle time (s)');
title('Settle time (x = gyro never went quiet)');

if opt.Save
    [dd, n] = fileparts(S.file);
    print(f1, fullfile(dd, [n '_tiltmap.png']), '-dpng', '-r150');
    print(f2, fullfile(dd, [n '_tiltsettle.png']), '-dpng', '-r150');
    fprintf('Saved %s_tiltmap.png and _tiltsettle.png in %s\n', n, dd);
end
end


function s = step(v)
s = min(diff(v));
if isempty(s) || ~isfinite(s) || s <= 0, s = 10; end
end


function M = to_grid(P, v, xs, ys, z, ok)
M = nan(numel(ys), numel(xs));
on = ok & abs(P(:, 3) - z) < 0.05 & isfinite(v);
[~, ix] = min(abs(P(on, 1) - xs.'), [], 2);
[~, iy] = min(abs(P(on, 2) - ys.'), [], 2);
M(sub2ind(size(M), iy, ix)) = v(on);
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
