function plot_field_arrows3d(dataFile, baselineFile, varargin)
% plot_field_arrows3d — 3D field arrows, one colour per z layer, on see-through planes.
%
%   plot_field_arrows3d(file)                 % raw field (includes Earth's field)
%   plot_field_arrows3d(file, baselineFile)   % file minus a reference scan, matched on
%                                             % position (magnet or coil field only)
%   plot_field_arrows3d(file, '', 'Scale', 2)        % arrows 2x longer
%   plot_field_arrows3d(file, '', 'Equal', true)     % all arrows the same length (direction only)
%   plot_field_arrows3d(file, '', 'Units', 'mm')     % axes in mm (default cm)
%   plot_field_arrows3d(file, '', 'Relative', false) % robot z instead of z from the middle layer
%   plot_field_arrows3d(file, '', 'Save', true)      % also save <name>_arrows3d.png next to the CSV
%   plot_field_arrows3d(file, '', 'R', R)            % sensor -> robot rotation
%                                                    % (default: scan_config's R_sensor_to_robot)
%   plot_field_arrows3d(file, '', 'TiltCorrect', true)     % remove the probe's tilt (pitch/roll)
%   plot_field_arrows3d(file, base, 'TiltCorrect', true)   % ... in both, to the baseline's centre
%
% 'TiltCorrect' needs a scan from field_scan_tilt.py (accelerometer columns). The
% field at every point is rotated back to the reference orientation (tilt_correct)
% before plotting. true: the reference is the baseline's centre point, or this
% scan's centre with no baseline. It can also be a reference: 'mean', another
% scan's file name, or a gravity vector [gx gy gz] (see tilt_correct).
%
% Arrow length is proportional to the field strength, scaled so a typical arrow
% (90th percentile of |B|) is one grid spacing long; stronger points (e.g. right
% next to a magnet) get longer arrows. 'Scale' multiplies that. Raw data is mostly
% Earth's field, so the arrows all point the same way; subtract a baseline scan
% to see the magnet's or the coils' own field.
%
% By default z is shown relative to the middle layer (0 = middle), x and y as
% scanned, all in cm. Positions are the TARGET grid points (x_mm..z_mm).

p = inputParser;
p.addParameter("Scale", 1);
p.addParameter("Equal", false);    % true: all arrows the same length (direction only)
p.addParameter("Units", "cm");
p.addParameter("Relative", true);
p.addParameter("Save", false);
p.addParameter("R", []);
p.addParameter("TiltCorrect", false);   % true: remove the probe's tilt first (apply_tilt)
p.parse(varargin{:});
opt = p.Results;
if nargin < 2, baselineFile = ''; end
dataFile = resolve_path(dataFile);
baselineFile = resolve_path(baselineFile);

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
[B, tiltTag] = apply_tilt(B, dataFile, opt.TiltCorrect, baselineFile);
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
    B0all = apply_tilt([T0.Bx_G T0.By_G T0.Bz_G], baselineFile, opt.TiltCorrect, baselineFile);
    B0(found, :) = B0all(loc(found), :);
    B = B - B0;
    [~, n0] = fileparts(char(baselineFile));
    what = ['Field minus ' n0];
end
what = [what tiltTag];
B = (R * B.').';                     % sensor axes -> robot axes
ok = all(isfinite(B), 2);
P = P(ok, :); B = B(ok, :);

zs = sort(unique(P(:, 3)), "descend");          % top layer first
z0 = 0;
if opt.Relative, z0 = median(unique(P(:, 3))); end
u = 1; ulab = 'mm';
if strcmpi(opt.Units, "cm"), u = 10; ulab = 'cm'; end

% arrow scale: 90th-percentile |B| -> one grid spacing (times 'Scale')
sp = [min(diff(unique(P(:, 1)))) min(diff(unique(P(:, 2))))];
sp = min(sp(isfinite(sp) & sp > 0));
if isempty(sp), sp = 10; end
Bn = sqrt(sum(B.^2, 2));
Bmax = max(Bn);
Bs = sort(Bn); Bref = Bs(max(1, ceil(0.9 * numel(Bs))));
if Bref == 0, Bref = 1; end
k = opt.Scale * sp / Bref;                      % mm per gauss
if opt.Equal
    B = B ./ max(Bn, eps) * Bref;               % unit direction, one spacing long
end

% layer colours, top to bottom: orange, blue, green, then the rest
C = [0.850 0.325 0.098; 0.000 0.447 0.741; 0.466 0.674 0.188; ...
     0.494 0.184 0.556; 0.929 0.694 0.125; 0.301 0.745 0.933; 0.635 0.078 0.184];

[~, name] = fileparts(char(dataFile));
fig = figure("Name", ['3D arrows: ' name], "Color", "w", "Position", [80 80 1000 700]);
hold on
hp = zeros(numel(zs), 1);
labels = cell(numel(zs), 1);
xr = [min(P(:, 1)) max(P(:, 1))] + [-0.5 0.5] * sp;
yr = [min(P(:, 2)) max(P(:, 2))] + [-0.5 0.5] * sp;
for i = 1:numel(zs)
    c = C(mod(i - 1, size(C, 1)) + 1, :);
    zp = (zs(i) - z0) / u;
    hp(i) = patch(xr([1 2 2 1]) / u, yr([1 1 2 2]) / u, zp * [1 1 1 1], c, ...
        "FaceAlpha", 0.12, "EdgeColor", c, "EdgeAlpha", 0.3);
    on = abs(P(:, 3) - zs(i)) < 0.05;
    quiver3(P(on, 1) / u, P(on, 2) / u, (P(on, 3) - z0) / u, ...
        k * B(on, 1) / u, k * B(on, 2) / u, k * B(on, 3) / u, 0, ...
        "Color", c, "LineWidth", 1.2, "MaxHeadSize", 0.3);
    labels{i} = sprintf('Z = %g mm', zs(i) - z0);
end
hold off
axis equal; grid on; box on
xlabel(sprintf('X (%s)', ulab)); ylabel(sprintf('Y (%s)', ulab)); zlabel(sprintf('Z (%s)', ulab));
view(-25, 20);
legend(hp, labels, "Location", "southeast");
zref = '';
if opt.Relative, zref = sprintf(', Z = 0 at robot z = %g mm', z0); end
if opt.Equal
    alab = 'arrows = direction only (equal length)';
else
    alab = sprintf('arrow length ~ |B|: one grid spacing = %.3f G (strongest %.3f G)', Bref / opt.Scale, Bmax);
end
title(sprintf('%s   (%s)\n%s%s', what, name, alab, zref), ...
    "Interpreter", "none");

if opt.Save
    [d, n] = fileparts(char(dataFile));
    print(fig, fullfile(d, [n '_arrows3d.png']), '-dpng', '-r150');
    fprintf("Saved %s_arrows3d.png in %s\n", n, d);
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
here = fileparts(mfilename("fullpath"));
alt = fullfile(here, f);
if isfile(alt), f = alt; return; end
error("Cannot find %s\nLooked in %s and in %s.\nList the scans with:  dir(fullfile('%s', 'data'))", ...
    f, pwd, here, here);
end
