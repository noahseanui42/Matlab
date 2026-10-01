function files = make_program(name, xyz, opts)
% make_program — write delta-app program files (File > Program > Open) that
% visit each point with a "Wait time" after it, so the data log has a clear
% stop at every point for compile_scan.
%
%   make_program("baseline")              % grid from scan_config.m
%   make_program("magnet", xyz)           % your own N x 3 points, probe mm
%   make_program("magnet", [], wait_ms=2000)
%
% The delta app takes at most 50 points per program, so bigger grids are
% split into name_1.txt, name_2.txt, ... (run them in order with the log on).

arguments
    name (1,1) string
    xyz double = []
    opts.wait_ms (1,1) double = 1500       % >= settle 0.5 s + ~1 s of samples
    opts.velocity_pct (1,1) double = 20    % 20% -> v=2 -> 10 mm/s
    opts.accel_pct (1,1) double = 10
    opts.max_points (1,1) double = 50      % delta app MAX_PROGRAM_LENGTH
end

here = fileparts(mfilename('fullpath'));
if isempty(xyz)
    cfg = scan_config();
    addpath(fullfile(here, "..", "Kinematics"));
    xyz = scan_grid(cfg.xr, cfg.yr, cfg.zr, cfg.nx, cfg.ny, cfg.nz);
end
xyz = round(xyz, 2);

outDir = fullfile(here, "programs");
if ~isfolder(outDir), mkdir(outDir); end

N = size(xyz, 1);
nChunks = ceil(N / opts.max_points);
files = strings(nChunks, 1);
for c = 1:nChunks
    k = (c-1)*opts.max_points + 1 : min(c*opts.max_points, N);
    n = numel(k);
    idx = num2cell(0:n-1);   % cells so a 1-point program still encodes as a JSON list

    pts.index_of_point = idx;
    pts.interpolation  = repmat({'Joint'}, 1, n);
    pts.velocity       = repmat({sprintf('%d%%', opts.velocity_pct)}, 1, n);
    pts.acceleration   = repmat({sprintf('%d%%', opts.accel_pct)}, 1, n);
    pts.coordinates    = struct('x', {num2cell(xyz(k,1)')}, ...
                                'y', {num2cell(xyz(k,2)')}, ...
                                'z', {num2cell(xyz(k,3)')});

    fn.func_type = repmat({'Wait time'}, 1, n);
    fn.pt_no     = idx;
    fn.value     = num2cell(repmat(opts.wait_ms, 1, n));
    fn.pin_no    = num2cell(zeros(1, n));

    if nChunks == 1
        files(c) = fullfile(outDir, name + ".txt");
    else
        files(c) = fullfile(outDir, sprintf("%s_%d.txt", name, c));
    end
    fid = fopen(files(c), "w");
    fprintf(fid, "%s\n%s", jsonencode(pts), jsonencode(fn));
    fclose(fid);
end

perPoint = opts.wait_ms/1000 + 2.5;   % wait + a typical 25 mm move at 10 mm/s
fprintf("%d points -> %d program file(s) in %s\n", N, nChunks, outDir);
fprintf("Roughly %.0f min of robot time.\n", N * perPoint / 60);
end
