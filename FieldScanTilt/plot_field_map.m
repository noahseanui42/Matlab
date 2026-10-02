function plot_field_map(dataFile, baselineFile, varargin)
% plot_field_map — plot a scan CSV from run_field_scan.
%
%   plot_field_map(file)                 % raw field (includes Earth's field)
%   plot_field_map(file, baselineFile)   % coil field only: file minus a coils-off
%                                        % scan, matched point by point on position
%   plot_field_map(file, '', 'TiltCorrect', true)     % remove the probe's tilt (pitch/roll)
%   plot_field_map(file, base, 'TiltCorrect', true)   % ... in both, to the baseline's centre
%
% 'TiltCorrect' needs a scan from field_scan_tilt.py (accelerometer columns); see
% apply_tilt and tilt_correct. It can also be a reference ('mean', a file, [gx gy gz]).
%
% Works on CSVs from field_scan.py and run_field_scan.m.
%
% Figure 1: field vectors, coloured by |B|
% Figure 2: |B| on slices through the centre of the scan volume
% Figure 3: % deviation of |B| from the centre value on the middle z plane
%           (the usual Helmholtz uniformity plot)

p = inputParser;
p.addParameter("TiltCorrect", false);   % true: remove the probe's tilt first (apply_tilt)
p.parse(varargin{:});
opt = p.Results;
if nargin < 2 || strlength(string(baselineFile)) == 0, baselineFile = ''; end

T = readtable(dataFile);
P = [T.x_mm T.y_mm T.z_mm];
[B, tiltTag] = apply_tilt([T.Bx_G T.By_G T.Bz_G], dataFile, opt.TiltCorrect, baselineFile);
B = B * 100;                        % gauss -> uT
what = "Raw field";

if ~isempty(baselineFile)
    T0 = readtable(baselineFile);
    % match points by position (to 0.1 mm), so skipped or reordered points still line up
    [found, loc] = ismember(round(P, 1), round([T0.x_mm T0.y_mm T0.z_mm], 1), 'rows');
    if ~any(found)
        error("Baseline has no points in common with the data file.");
    elseif ~all(found)
        warning("%d of %d points have no baseline at the same position and are left out.", ...
            nnz(~found), numel(found));
    end
    B0 = nan(size(B));
    B0all = apply_tilt([T0.Bx_G T0.By_G T0.Bz_G], baselineFile, opt.TiltCorrect, baselineFile);
    B0(found,:) = B0all(loc(found),:) * 100;
    B = B - B0;
    what = "Coil field (baseline subtracted)";
end
what = what + tiltTag;

ok = all(isfinite(B), 2);
P = P(ok,:);  B = B(ok,:);
Bmag = vecnorm(B, 2, 2);
[~, name] = fileparts(dataFile);

% --- 1: vectors ---
figure('Name', "Vectors: " + name);
scatter3(P(:,1), P(:,2), P(:,3), 30, Bmag, 'filled'); hold on
quiver3(P(:,1), P(:,2), P(:,3), B(:,1), B(:,2), B(:,3), 'k');
hold off; axis equal; grid on
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
cb = colorbar; cb.Label.String = '|B| (\muT)';
title(what + ": field vectors");

% --- regular grid for slices (the scan grid itself, interpolated over gaps) ---
xs = unique(P(:,1)); ys = unique(P(:,2)); zs = unique(P(:,3));
if numel(xs) < 2 || numel(ys) < 2 || numel(zs) < 2
    warning("Need at least 2 points along x, y and z for slice plots.");
    return
end
F = scatteredInterpolant(P(:,1), P(:,2), P(:,3), Bmag, 'linear', 'none');
[X, Y, Z] = meshgrid(xs, ys, zs);
V = F(X, Y, Z);
c = [mean(xs) mean(ys) mean(zs)];

% --- 2: |B| slices ---
figure('Name', "Slices: " + name);
h = slice(X, Y, Z, V, c(1), c(2), c(3));
set(h, 'EdgeColor', 'none', 'FaceColor', 'interp');
axis equal; grid on
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
cb = colorbar; cb.Label.String = '|B| (\muT)';
title(what + ": |B| through the scan centre");

% --- 3: uniformity on the middle z plane ---
Bc = F(c(1), c(2), c(3));
if ~isfinite(Bc), Bc = median(Bmag); end
[~, iz] = min(abs(zs - c(3)));
dev = 100 * (V(:,:,iz) - Bc) / Bc;
figure('Name', "Uniformity: " + name);
contourf(xs, ys, dev, 20, 'LineColor', 'none');
axis equal; grid on
xlabel('x (mm)'); ylabel('y (mm)');
cb = colorbar; cb.Label.String = 'deviation from centre (%)';
title(sprintf("%s: |B| deviation at z = %.0f mm (centre %.2f \\muT)", what, zs(iz), Bc));
end
