function [dataFile, baselineFile] = make_demo_scan()
% make_demo_scan — write a fake coils-on scan and a fake baseline scan
% (Helmholtz pair by Biot-Savart + a constant Earth field) on the
% scan_config.m grid, with the tilt columns, so the plots can be tried with
% no hardware:
%
%   [f, f0] = make_demo_scan();  plot_field_map(f, f0)
%   plot_tilt(f)                            % tilt, pitch/roll and gyro per point
%   tilt_correct(f0, 'Write', true);        % both to the same reference, then subtract
%   tilt_correct(f, 'Reference', f0, 'Write', true);
%
% The platform tips outwards by 0.01 deg per mm off the axis (as the simulated
% sensor in field_scan_tilt.py), so both the field and gravity are seen rotated
% in the sensor's axes; tilt_correct takes that back out.

cfg = scan_config();
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, "..", "Kinematics"));
xyz = scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz);
N = size(xyz, 1);

R  = 0.30;                         % coil radius, m (demo value)
NI = 50;                           % amp-turns per coil (demo value)
zc = mean(cfg.zr) / 1000;          % coil pair centred on the scan volume, m
earth = [0.20 0.05 -0.50];         % G, roughly Wellington

Bcoil = zeros(N, 3);
for zLoop = zc + [-R/2, R/2]
    Bcoil = Bcoil + loop_field(xyz / 1000, R, zLoop, NI);
end

if ~isfolder(cfg.out_dir), mkdir(cfg.out_dir); end
dataFile     = fullfile(cfg.out_dir, "demo_coils_on.csv");
baselineFile = fullfile(cfg.out_dir, "demo_baseline.csv");
write_scan(dataFile,     xyz, Bcoil + earth);
write_scan(baselineFile, xyz, repmat(earth, N, 1));
end

function write_scan(f, xyz, Bworld)
% One row per point with field_scan_tilt.py's columns. The sensor sits on a
% platform tilted by k deg per mm off the axis: R = roty(k x) * rotx(-k y), so a
% world vector v reads as R.' * v in the sensor's axes. Gravity reads (0, 0, -1) g level.
k = 0.01;
N = size(xyz, 1);
cols = {'idx','x_mm','y_mm','z_mm','Bx_G','By_G','Bz_G','Bx_std_G','By_std_G','Bz_std_G', ...
        'err','t_s','n_samples','deg1','deg2','deg3','sent_x_mm','sent_y_mm','sent_z_mm', ...
        'ax_g','ay_g','az_g','ax_std_g','ay_std_g','az_std_g', ...
        'gyro_rms_dps','gyro_max_dps','settle_s','settled', ...
        'pitch_deg','roll_deg','acc_pitch_deg','acc_roll_deg', ...
        'spatial_t_first_ms','spatial_t_last_ms'};
M = zeros(N, numel(cols));
for i = 1:N
    R = roty(k * xyz(i,1)) * rotx(-k * xyz(i,2));
    B = (R.' * Bworld(i,:).').';
    a = (R.' * [0; 0; -1]).';
    s = 1; if a(3) < 0, s = -1; end
    accPR = [atan2d(-s * a(1), hypot(a(2), a(3))), atan2d(s * a(2), abs(a(3)))];
    t = 6 * i;                                  % ~5 s settle + move per point
    g = 0.08 + 0.02 * sin(i);                   % gyro noise while still, deg/s
    M(i,:) = [i, xyz(i,:), B, 0.0008 * [1 1 1], 0, t, 20, nan(1,3), xyz(i,:), ...
              a, 0.0003 * [1 1 1], g, 2 * g, 5, -1, ...
              k * xyz(i,1), -k * xyz(i,2), accPR, 1000 * t - 400, 1000 * t - 20];
end
fid = fopen(f, 'w');
fprintf(fid, '%s\n', strjoin(cols, ','));
fprintf(fid, [strjoin(repmat({'%.10g'}, 1, numel(cols)), ','), '\n'], M.');
fclose(fid);
end

function R = rotx(deg)
c = cosd(deg); s = sind(deg);
R = [1 0 0; 0 c -s; 0 s c];
end

function R = roty(deg)
c = cosd(deg); s = sind(deg);
R = [c 0 s; 0 1 0; -s 0 c];
end

function B = loop_field(P, R, z0, NI)
% Biot-Savart for a circular loop in the x-y plane at height z0; P in m, B in G.
M = 360;
ph = (0:M-1)' * 2*pi/M;
L  = [R*cos(ph), R*sin(ph), z0*ones(M,1)];          % loop points
dl = [-R*sin(ph), R*cos(ph), zeros(M,1)] * 2*pi/M;  % segment vectors
B = zeros(size(P,1), 3);
for k = 1:size(P,1)
    r = P(k,:) - L;
    B(k,:) = sum(cross(dl, r, 2) ./ vecnorm(r, 2, 2).^3, 1);
end
B = B * 1e-7 * NI * 1e4;   % mu0/4pi * NI, tesla -> gauss
end
