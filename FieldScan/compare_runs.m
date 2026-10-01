function R = compare_runs(magnetFiles, noiseFile)
% compare_runs — repeatability of a fixed reference magnet across repeated scans.
%
%   R = compare_runs(magnetFiles)              % magnetFiles: string array / cell of CSVs
%   R = compare_runs("data/magnet_run*.csv")   % or a wildcard pattern
%   R = compare_runs(magnetFiles, noiseFile)   % + a magnet-absent scan for the noise floor
%
% Run the same grid several times with the magnet fixed
% (run_field_scan("magnet_run"+k)), plus one run without the magnet, then:
%
%   sigma_B   spread of |B| across runs at each point (std over runs, G)
%   sigma_pos position repeatability, sigma_B / |grad|B||, mm, using the gradient
%             of the run-averaged |B| on the grid
%
% The noise floor (sensor noise from the magnet-absent scan, in quadrature)
% is subtracted from sigma_B before converting to position. sigma_pos is only
% the displacement along the gradient direction, and it blows up where the
% gradient is small, so those points are flagged as not meaningful.
%
% Figures: 1) sigma_B per point vs noise floor, 2) sigma_pos on the grid,
% 3) histogram of sigma_pos. R is a struct of the numbers.

cfg = scan_config();

files = string(magnetFiles);
if isscalar(files) && contains(files, ["*" "?"])
    d = dir(files);
    files = string(fullfile({d.folder}, {d.name}));
end
files = files(:);
nRun = numel(files);
if nRun < 2, error("Need at least 2 magnet runs, got %d.", nRun); end

% --- load runs; all must be on the same grid ---
P = [];  Bruns = [];
for r = 1:nRun
    T = readtable(files(r));
    Pr = [T.x_mm T.y_mm T.z_mm];
    if r == 1
        P = Pr;
        Bruns = nan(height(T), 3, nRun);
    elseif size(Pr,1) ~= size(P,1) || any(abs(Pr - P) > 1e-6, 'all')
        error("%s is not on the same grid as %s.", files(r), files(1));
    end
    Bruns(:,:,r) = [T.Bx_G T.By_G T.Bz_G];
end
N = size(P, 1);

Bmag_runs = squeeze(vecnorm(Bruns, 2, 2));      % N x nRun
if N == 1, Bmag_runs = Bmag_runs(:).'; end
nValid = sum(isfinite(Bmag_runs), 2);
Bmean = mean(Bruns, 3, 'omitnan');              % N x 3
Bmag  = vecnorm(Bmean, 2, 2);
sigB  = std(Bmag_runs, 0, 2, 'omitnan');        % N x 1, over runs
sigB(nValid < 2) = NaN;

% --- sensor noise floor from the magnet-absent scan ---
sigNoise = zeros(N, 1);
haveNoise = nargin > 1 && strlength(string(noiseFile)) > 0;
if haveNoise
    T0 = readtable(noiseFile);
    P0 = [T0.x_mm T0.y_mm T0.z_mm];
    if size(P0,1) ~= N || any(abs(P0 - P) > 1e-6, 'all')
        error("Noise scan was not taken on the same grid as the magnet runs.");
    end
    B0  = [T0.Bx_G T0.By_G T0.Bz_G];
    sd0 = [T0.Bx_std_G T0.By_std_G T0.Bz_std_G] / sqrt(cfg.n_avg);  % std of each point's mean
    u0  = B0 ./ max(vecnorm(B0, 2, 2), eps);                        % unit vector of B
    sigNoise = sqrt(sum((u0 .* sd0).^2, 2));                        % noise in |B|, G
end
sigB_corr = sqrt(max(sigB.^2 - sigNoise.^2, 0));

% --- gradient of the run-averaged |B| on the regular grid ---
[xs, ~, ix] = unique(round(P(:,1), 2));
[ys, ~, iy] = unique(round(P(:,2), 2));
[zs, ~, iz] = unique(round(P(:,3), 2));
if numel(xs) < 2 || numel(ys) < 2 || numel(zs) < 2
    error("Need at least 2 grid points along each of x, y and z for the gradient.");
end
idx = sub2ind([numel(xs) numel(ys) numel(zs)], ix, iy, iz);
V = nan(numel(xs), numel(ys), numel(zs));
V(idx) = Bmag;
[Gx, Gy, Gz] = gradient(V, xs, ys, zs);         % G per mm; NaN where neighbours are missing
gradMag = vecnorm([Gx(idx) Gy(idx) Gz(idx)], 2, 2);

sigPos = sigB_corr ./ gradMag;                  % mm
% A gradient below the noise-limited resolution tells us nothing about position.
minGrad = median(gradMag, 'omitnan') * 0.1;
weak = ~(gradMag > minGrad);
sigPos(weak) = NaN;

R = struct('files', files, 'P', P, 'Bmean', Bmean, 'sigma_B', sigB, ...
           'sigma_noise', sigNoise, 'sigma_B_corrected', sigB_corr, ...
           'grad', gradMag, 'sigma_pos_mm', sigPos, 'weak_gradient', weak);

% --- summary ---
fprintf("\n%d runs, %d points, %d with a usable gradient.\n", nRun, N, sum(~weak & isfinite(sigPos)));
fprintf("sigma_B    : median %.5f G, max %.5f G\n", median(sigB, 'omitnan'), max(sigB));
if haveNoise
    fprintf("noise floor: median %.5f G (from %s)\n", median(sigNoise), noiseFile);
else
    fprintf("noise floor: not supplied, sigma_B includes sensor noise (upper bound).\n");
end
fprintf("sigma_pos  : median %.3f mm, 95th percentile %.3f mm, max %.3f mm\n", ...
    median(sigPos, 'omitnan'), prctile(sigPos(isfinite(sigPos)), 95), max(sigPos));
fprintf("Note: sigma_B also contains field drift between runs (Earth/coil/temperature),\n");
fprintf("so sigma_pos is an upper bound on the robot's true position repeatability.\n");

% --- plots ---
figure('Name', "Repeatability: sigma_B");
plot(1:N, sigB * 1e5, 'o-'); hold on     % 1 G = 1e5 nT
if haveNoise, plot(1:N, sigNoise * 1e5, 's-'); end
hold off; grid on
xlabel('point index'); ylabel('\sigma_{|B|} (nT)');
if haveNoise, legend('across runs', 'sensor noise floor'); else, legend('across runs'); end
title(sprintf("Spread of |B| over %d runs", nRun));

figure('Name', "Repeatability: sigma_pos");
ok = isfinite(sigPos);
scatter3(P(ok,1), P(ok,2), P(ok,3), 60, sigPos(ok), 'filled'); hold on
if any(~ok), plot3(P(~ok,1), P(~ok,2), P(~ok,3), 'kx'); end
hold off; axis equal; grid on
xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
cb = colorbar; cb.Label.String = '\sigma_{pos} (mm)';
title("Position repeatability  \sigma_B / |\nabla B|   (x = no usable gradient)");

figure('Name', "Repeatability: histogram");
histogram(sigPos(ok));
grid on; xlabel('\sigma_{pos} (mm)'); ylabel('points');
title(sprintf("median %.3f mm", median(sigPos, 'omitnan')));
end
