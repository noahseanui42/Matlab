% compile_scan_results.m — run after a physical scan to compile the
% magnetometer log against the planned grid from pulse_table.mat
%
% expects a post-run log with columns: idx,Bx,By,Bz (idx matches the
% pulse_table 'idx' column, i.e. the raster order the robot was driven in)

%% --- config ---
results_csv = 'scan_results.csv';     % post-run magnetometer log
bg_csv      = '';                     % optional zero-field background log, same columns; '' to skip
sensor_sens = 1;                      % counts -> uT  <-- MEASURE/SET THIS (1 if log is already calibrated)

%% --- load the planned grid ---
load('pulse_table.mat', 'T', 'meta');
nx = meta.volume.nx;  ny = meta.volume.ny;  nz = meta.volume.nz;
xr = meta.volume.xr;  yr = meta.volume.yr;  zr = meta.volume.zr;
N  = nx*ny*nz;

xs = linspace(xr(1), xr(2), nx);
ys = linspace(yr(1), yr(2), ny);
zs = linspace(zr(1), zr(2), nz);

%% --- load post-run measurements ---
R = readtable(results_csv);
req = {'idx','Bx','By','Bz'};
assert(all(ismember(req, R.Properties.VariableNames)), ...
    'scan_results.csv must have columns: %s', strjoin(req, ','));

[dup, ~] = groupcounts(R.idx);
if any(dup > 1)
    warning('scan_results.csv has duplicate idx rows — keeping the first of each');
    [~, first] = unique(R.idx, 'stable');
    R = R(first,:);
end

%% --- join measurements onto the planned grid by idx ---
[tf, loc] = ismember(T.idx, R.idx);
if ~all(tf)
    warning('%d of %d planned points have no measurement row', sum(~tf), N);
end

Bx = nan(N,1);  By = nan(N,1);  Bz = nan(N,1);
Bx(tf) = R.Bx(loc(tf));
By(tf) = R.By(loc(tf));
Bz(tf) = R.Bz(loc(tf));

%% --- drop points the IK/validity pass already flagged unreachable ---
bad = tf & ~T.valid;
if any(bad)
    warning('%d measured points were flagged invalid by validate_batch — discarding', sum(bad));
end
Bx(~T.valid) = NaN;  By(~T.valid) = NaN;  Bz(~T.valid) = NaN;

%% --- optional background (ambient field) subtraction ---
if ~isempty(bg_csv)
    BG = readtable(bg_csv);
    [btf, bloc] = ismember(T.idx, BG.idx);
    Bx(btf) = Bx(btf) - BG.Bx(bloc(btf));
    By(btf) = By(btf) - BG.By(bloc(btf));
    Bz(btf) = Bz(btf) - BG.Bz(bloc(btf));
end

%% --- calibrate to physical units ---
Bx = Bx * sensor_sens;   % uT
By = By * sensor_sens;
Bz = Bz * sensor_sens;
Bmag = sqrt(Bx.^2 + By.^2 + Bz.^2);

%% --- scatter raster-order rows into an ascending-axis [nx,ny,nz] grid ---
% scan_grid.m drives x in reverse on every other y-row (boustrophedon);
% idx is just linear raster order, so recover (ix,jy,iz) from it and
% un-reverse the even rows to land each sample at its physical (xs,ys,zs)
% cell instead of its visit order.
Xg  = nan(nx,ny,nz);  Yg  = nan(nx,ny,nz);  Zg  = nan(nx,ny,nz);
BXg = nan(nx,ny,nz);  BYg = nan(nx,ny,nz);  BZg = nan(nx,ny,nz);
BMg = nan(nx,ny,nz);

[ix, jy, iz] = ind2sub([nx, ny, nz], T.idx);
xi = ix;
rev = mod(jy,2) == 0;
xi(rev) = nx - ix(rev) + 1;

for k = 1:N
    Xg(xi(k),jy(k),iz(k))  = xs(xi(k));
    Yg(xi(k),jy(k),iz(k))  = ys(jy(k));
    Zg(xi(k),jy(k),iz(k))  = zs(iz(k));
    BXg(xi(k),jy(k),iz(k)) = Bx(k);
    BYg(xi(k),jy(k),iz(k)) = By(k);
    BZg(xi(k),jy(k),iz(k)) = Bz(k);
    BMg(xi(k),jy(k),iz(k)) = Bmag(k);
end

%% --- uniformity vs. the volume centre ---
[~, cx] = min(abs(xs - mean(xr)));
[~, cy] = min(abs(ys - mean(yr)));
[~, cz] = min(abs(zs - mean(zr)));
B0 = BMg(cx,cy,cz);
pct_dev = 100 * (BMg - B0) / B0;

%% --- visualize ---
figure('Name','Field magnitude by z-slice');
for iz_ = 1:nz
    subplot(ceil(nz/ceil(sqrt(nz))), ceil(sqrt(nz)), iz_);
    imagesc(xs, ys, squeeze(BMg(:,:,iz_))');
    axis xy equal tight;  colorbar;
    title(sprintf('z = %.0f mm', zs(iz_)));
    xlabel('x (mm)');  ylabel('y (mm)');
end

figure('Name','Field magnitude, 3D scatter');
scatter3(Xg(:), Yg(:), Zg(:), 25, BMg(:), 'filled');
axis equal;  colorbar;  xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
title('|B| over the scan volume');

figure('Name','Field vector, mid z-slice');
quiver3(Xg(:,:,cz), Yg(:,:,cz), Zg(:,:,cz), BXg(:,:,cz), BYg(:,:,cz), BZg(:,:,cz));
axis equal;  xlabel('x (mm)'); ylabel('y (mm)'); zlabel('z (mm)');
title(sprintf('B vector field, z = %.0f mm', zs(cz)));

%% --- export ---
results = struct('Xg',Xg,'Yg',Yg,'Zg',Zg, 'BXg',BXg,'BYg',BYg,'BZg',BZg, ...
    'BMg',BMg, 'B0',B0, 'pct_dev',pct_dev, 'meta',meta, 'built',datetime('now'));
save('scan_results_compiled.mat','results');

Tout = table(Xg(:), Yg(:), Zg(:), BXg(:), BYg(:), BZg(:), BMg(:), pct_dev(:), ...
    'VariableNames', {'x','y','z','Bx','By','Bz','Bmag','pct_dev'});
writetable(Tout, 'scan_results_compiled.csv');
