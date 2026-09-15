% init_scan.m — run before opening/simulating S01_Scan_Pipeline

% --- geometry (locked) ---
sb = 175;   % base triangle side, mm
sp = 150;   % platform triangle side, mm
L1 = 180;   % proximal (bicep), mm
L2 = 600;   % distal (forearm), mm   <-- 600 vs 700 still unresolved

% --- scan volume ---
xr = [-100 100];
yr = [ -50  50];
zr = [-650 -550];

% --- grid counts, NOT step sizes (see Gotcha 1) ---
nx = 21;  ny = 11;  nz = 11;
N  = nx*ny*nz;        % 2541 points

% --- servo calibration, per axis, from servo_calibration_v3 ---
pw_min  = [830 830 830];      % us, working limits
pw_max  = [2170 2170 2170];
pw_home = [1500 1500 1500];   % us at theta = 0 (bicep horizontal)
k_us    = [11.67 11.67 11.67]; % us per degree  <-- MEASURE THESE
sgn     = [1 -1 -1];           % direction per axis <-- CONFIRM THESE Should be same dir just need to test - or +

% --- validation limits ---
artic_max = 13;               % SIL6T/K rod end misalignment limit, deg
rod_tol   = 1e-6;             % mm, forearm length residual
th_lim    = [-45 90];         % deg, from pw limits via k_us — recompute

% --- nominal ball-stud axes (pre-angled to volume centre) ---
nhat = rod_axes_at([0 0 -600], sb, sp, L1, L2);
