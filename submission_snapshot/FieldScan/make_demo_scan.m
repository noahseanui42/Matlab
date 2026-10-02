function [dataFile, baselineFile] = make_demo_scan()
% make_demo_scan — write a fake coils-on scan and a fake baseline scan
% (Helmholtz pair by Biot-Savart + a constant Earth field) on the
% scan_config.m grid, so plot_field_map can be tried with no hardware:
%
%   [f, f0] = make_demo_scan();  plot_field_map(f, f0)

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

function write_scan(f, xyz, B)
N = size(xyz, 1);
T = table((1:N)', xyz(:,1), xyz(:,2), xyz(:,3), B(:,1), B(:,2), B(:,3), ...
    zeros(N,1), zeros(N,1), zeros(N,1), zeros(N,1), zeros(N,1), ...
    'VariableNames', {'idx','x_mm','y_mm','z_mm','Bx_G','By_G','Bz_G', ...
                      'Bx_std_G','By_std_G','Bz_std_G','err','t_s'});
writetable(T, f);
end
