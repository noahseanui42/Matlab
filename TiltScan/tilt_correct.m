function S = tilt_correct(dataFile, varargin)
% tilt_correct — tilt of the probe at every point of a field_scan_tilt.py scan, from
% the accelerometer, and the field rotated back to a reference orientation.
%
%   S = tilt_correct(file)                          % tilt relative to the grid centre
%   S = tilt_correct(file, 'Write', true)           % also write <name>_tiltcorr.csv
%   S = tilt_correct(file, 'Reference', otherFile)  % use another scan's centre as the reference
%   S = tilt_correct(file, 'Reference', 'mean')     % ... or the mean gravity direction of the scan
%   S = tilt_correct(file, 'Reference', [gx gy gz]) % ... or a given direction (sensor axes)
%
% While the probe is still the accelerometer measures gravity only, so the unit
% vector g = a / |a| is the vertical seen in the sensor's axes. At each point:
%
%   tilt_deg       angle between g and the reference direction g_ref
%   tilt_vec_deg   the same as a rotation vector (axis x angle, deg, sensor axes):
%                  the rotation that takes g onto g_ref
%   B_corr_G       B rotated by that rotation, i.e. the field the sensor would have
%                  read at the reference orientation
%
% Only tilt (roll and pitch) can be seen this way. A turn about the vertical leaves
% gravity unchanged, so it is neither measured nor corrected.
%
% For a magnet scan and its no-magnet background, correct both to the SAME
% reference, then subtract:
%
%   tilt_correct(bgFile, 'Write', true);
%   tilt_correct(magFile, 'Reference', bgFile, 'Write', true);
%   fit_dipole(<mag>_tiltcorr.csv, <bg>_tiltcorr.csv)   % or plot_field_layers(...)
%
% The written CSV is the input with Bx_G..Bz_G replaced by the corrected field and
% Bx_raw_G, By_raw_G, Bz_raw_G, tilt_deg added at the end, so every FieldScan
% script reads it unchanged. The .meta.json is copied alongside. Standard
% deviations are left as measured (the rotation is far too small to matter).
% Units: G, g, deg, sensor axes (apply R_sensor_to_robot afterwards, as the
% FieldScan scripts do).

p = inputParser;
p.addParameter('Reference', 'centre');
p.addParameter('Write', false);
p.parse(varargin{:});
opt = p.Results;
dataFile = resolve_path(dataFile);

T = read_scan(dataFile);
if ~isfield(T, 'ax_g')
    error('%s has no accelerometer columns (ax_g..az_g). Was it recorded with field_scan_tilt.py?', dataFile);
end
P = [T.x_mm T.y_mm T.z_mm];
A = [T.ax_g T.ay_g T.az_g];
B = [T.Bx_G T.By_G T.Bz_G];
N = size(P, 1);
ok = all(isfinite(A), 2) & sqrt(sum(A.^2, 2)) > 0.5;   % still and read: |a| close to 1 g
g = nan(N, 3);
g(ok, :) = A(ok, :) ./ sqrt(sum(A(ok, :).^2, 2));
gref = reference(opt.Reference, P, g, ok);

% rotation taking g onto g_ref at each point (Rodrigues), applied to B
tiltDeg = nan(N, 1);
tiltVec = nan(N, 3);
Bc = nan(N, 3);
for i = find(ok).'
    ax = cross(g(i, :), gref);
    s = norm(ax);
    c = max(-1, min(1, dot(g(i, :), gref)));
    th = atan2(s, c);
    tiltDeg(i) = rad2deg(th);
    if s < 1e-12
        k = [0 0 0];
    else
        k = ax / s;
    end
    tiltVec(i, :) = rad2deg(th) * k;
    v = B(i, :);
    Bc(i, :) = v * cos(th) + cross(k, v) * sin(th) + k * dot(k, v) * (1 - cos(th));
end

S = struct();
S.file = dataFile;
S.P_mm = P;
S.g = g;
S.g_ref = gref;
S.tilt_deg = tiltDeg;
S.tilt_vec_deg = tiltVec;
S.B_raw_G = B;
S.B_corr_G = Bc;
S.ok = ok;
S.out_file = '';

d = tiltDeg(ok);
fprintf('%s: tilt relative to the reference, %d points: median %.3f deg, max %.3f deg\n', ...
    short_name(dataFile), nnz(ok), median(d), max(d));
dB = sqrt(sum((Bc(ok, :) - B(ok, :)).^2, 2));
fprintf('  change in B from the correction: median %.5f G, max %.5f G\n', median(dB), max(dB));

if opt.Write
    [dd, n] = fileparts(dataFile);
    S.out_file = fullfile(dd, [n '_tiltcorr.csv']);
    write_corrected(S.out_file, T, Bc, B, tiltDeg);
    mf = fullfile(dd, [n '.meta.json']);
    if isfile(mf)
        copyfile(mf, fullfile(dd, [n '_tiltcorr.meta.json']));
    end
    fprintf('  wrote %s\n', S.out_file);
end
end


function gref = reference(ref, P, g, ok)
if isnumeric(ref)
    gref = ref(:).' / norm(ref);
    return
end
ref = char(ref);
switch lower(ref)
    case 'mean'
        gref = mean(g(ok, :), 1);
        gref = gref / norm(gref);
    case {'centre', 'center'}
        gref = centre_g(P, g, ok);
    otherwise   % another scan: its centre point
        T0 = read_scan(resolve_path(ref));
        if ~isfield(T0, 'ax_g'), error('Reference scan %s has no accelerometer columns.', ref); end
        A0 = [T0.ax_g T0.ay_g T0.az_g];
        ok0 = all(isfinite(A0), 2) & sqrt(sum(A0.^2, 2)) > 0.5;
        g0 = nan(size(A0));
        g0(ok0, :) = A0(ok0, :) ./ sqrt(sum(A0(ok0, :).^2, 2));
        gref = centre_g([T0.x_mm T0.y_mm T0.z_mm], g0, ok0);
end
end


function gref = centre_g(P, g, ok)
% gravity direction at the valid point nearest the middle of the grid
if ~any(ok), error('No valid accelerometer readings.'); end
c = (min(P(ok, :), [], 1) + max(P(ok, :), [], 1)) / 2;
idx = find(ok);
[~, j] = min(sum((P(idx, :) - c).^2, 2));
gref = g(idx(j), :);
end


function write_corrected(f, T, Bc, B, tiltDeg)
names = fieldnames(T).';
cols = cell(1, numel(names));
for i = 1:numel(names)
    cols{i} = T.(names{i});
end
cols{strcmp(names, 'Bx_G')} = Bc(:, 1);
cols{strcmp(names, 'By_G')} = Bc(:, 2);
cols{strcmp(names, 'Bz_G')} = Bc(:, 3);
names = [names {'Bx_raw_G', 'By_raw_G', 'Bz_raw_G', 'tilt_deg'}];
cols = [cols {B(:, 1), B(:, 2), B(:, 3), tiltDeg}];
M = [cols{:}];
fid = fopen(f, 'w');
if fid < 0, error('Cannot write %s', f); end
fprintf(fid, '%s\n', strjoin(names, ','));
fmt = [strjoin(repmat({'%.10g'}, 1, numel(names)), ','), '\n'];
fprintf(fid, fmt, M.');
fclose(fid);
end


function n = short_name(f)
[~, n] = fileparts(f);
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
% Accept a path relative to the current folder or to this script's folder (TiltScan).
f = char(f);
if isempty(f) || isfile(f), return; end
here = fileparts(mfilename('fullpath'));
alt = fullfile(here, f);
if isfile(alt), f = alt; return; end
error('Cannot find %s\nLooked in %s and in %s.', f, pwd, here);
end
