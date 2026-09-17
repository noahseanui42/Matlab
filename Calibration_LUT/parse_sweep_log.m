function out = parse_sweep_log(log_file, out_dir)
%PARSE_SWEEP_LOG Split a captured servo_calibration_sweep.ino serial log into CSVs.
%   out = parse_sweep_log(log_file, out_dir)
%
%   Reads the raw Serial Monitor capture of one or more sweeps (save the whole
%   session, prompts and all - nothing needs stripping by hand) and writes, for
%   every servo N that appears in it:
%
%     cal_servoN_fwd.csv      forward sweep        -> calibrate_servos_lut.m (Phase 2)
%     cal_servoN_bwd.csv      backward sweep       -> calibrate_servos_lut.m (Phase 2)
%     cal_servoN.csv          fwd + bwd, combined  -> Kinematics/calibrate_servos.m (Phase 1)
%     cal_servoN_holdout.csv  'P<us>' check points -> compare_calibrations.m (step 12)
%
%   Every CSV is [pulse_width_us, angle_deg], no header, one row per repeat
%   reading. The combined file is what the Phase 1 linear fit wants (it is
%   direction-blind); the fwd/bwd split is what the Phase 2 table wants.
%
%   Sections are recognised from the sketch's own markers, so the log can hold
%   all three servos back to back:
%     # BEGIN FORWARD SWEEP servo=N   ...   # END SWEEP
%     # BEGIN BACKWARD SWEEP servo=N  ...   # END SWEEP
%     # BEGIN HOLDOUT servo=N         ...   # END HOLDOUT
%   Bare "pw,angle" data lines outside any section are skipped with a warning.
%   An Arduino Serial Monitor "HH:MM:SS.mmm -> " timestamp prefix, if the
%   timestamp option was on, is stripped automatically.
%
%   out is a struct array, one element per servo, fields: servo, fwd, bwd,
%   combined, holdout (file paths; '' if that section was absent) and n_fwd,
%   n_bwd, n_holdout (row counts).

if nargin < 2 || isempty(out_dir)
    out_dir = fileparts(log_file);
    if isempty(out_dir), out_dir = pwd; end
end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

lines = readlines(log_file);

servos = struct('fwd', {}, 'bwd', {}, 'holdout', {});   % indexed by servo number
mode   = '';      % 'fwd' | 'bwd' | 'holdout' | ''
cur    = 0;       % servo number of the open section
n_orphan = 0;

for L = 1:numel(lines)
    s = strtrim(char(lines(L)));
    s = regexprep(s, '^\d{1,2}:\d{2}:\d{2}\.\d{3}\s*->\s*', '');   % Serial Monitor timestamp
    if isempty(s), continue; end

    tok = regexp(s, '^#\s*BEGIN\s+(FORWARD|BACKWARD)\s+SWEEP\s+servo=(\d+)', 'tokens', 'once');
    if ~isempty(tok)
        if strcmpi(tok{1}, 'FORWARD'), mode = 'fwd'; else, mode = 'bwd'; end
        cur = str2double(tok{2});
        continue
    end
    tok = regexp(s, '^#\s*BEGIN\s+HOLDOUT\s+servo=(\d+)', 'tokens', 'once');
    if ~isempty(tok)
        mode = 'holdout'; cur = str2double(tok{1});
        continue
    end
    if ~isempty(regexp(s, '^#\s*END\s+(SWEEP|HOLDOUT)', 'once'))
        mode = ''; cur = 0;
        continue
    end
    if s(1) == '#', continue; end   % prompt / status line

    tok = regexp(s, '^(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)$', 'tokens', 'once');
    if isempty(tok)
        continue   % operator echo or garbage - not a data line
    end
    row = [str2double(tok{1}), str2double(tok{2})];

    if isempty(mode)
        n_orphan = n_orphan + 1;
        continue
    end
    if cur > numel(servos), servos(cur).fwd = []; end   % grow the struct array first
    servos(cur).(mode)(end+1, :) = row;
end

if n_orphan > 0
    warning('parse_sweep_log:orphanData', ...
        '%d data line(s) found outside any BEGIN/END section were skipped.', n_orphan);
end

out = struct('servo', {}, 'fwd', {}, 'bwd', {}, 'combined', {}, 'holdout', {}, ...
             'n_fwd', {}, 'n_bwd', {}, 'n_holdout', {});
for N = 1:numel(servos)
    sv = servos(N);
    if isempty(sv.fwd) && isempty(sv.bwd) && isempty(sv.holdout), continue; end
    o = struct('servo', N, 'fwd', '', 'bwd', '', 'combined', '', 'holdout', '', ...
               'n_fwd', size(sv.fwd,1), 'n_bwd', size(sv.bwd,1), 'n_holdout', size(sv.holdout,1));

    if o.n_fwd > 0
        o.fwd = fullfile(out_dir, sprintf('cal_servo%d_fwd.csv', N));
        writematrix(sv.fwd, o.fwd);
    end
    if o.n_bwd > 0
        o.bwd = fullfile(out_dir, sprintf('cal_servo%d_bwd.csv', N));
        writematrix(sv.bwd, o.bwd);
    end
    if o.n_fwd > 0 || o.n_bwd > 0
        o.combined = fullfile(out_dir, sprintf('cal_servo%d.csv', N));
        writematrix([sv.fwd; sv.bwd], o.combined);
    end
    if o.n_holdout > 0
        o.holdout = fullfile(out_dir, sprintf('cal_servo%d_holdout.csv', N));
        writematrix(sv.holdout, o.holdout);
    end
    if xor(o.n_fwd > 0, o.n_bwd > 0)
        if o.n_fwd > 0, have = 'forward'; else, have = 'backward'; end
        warning('parse_sweep_log:oneDirection', ...
            'Servo %d has only a %s sweep - calibrate_servos_lut.m needs both to measure backlash.', N, have);
    end
    out(end+1) = o; %#ok<AGROW>

    fprintf('Servo %d: fwd=%d rows, bwd=%d rows, holdout=%d rows -> %s\n', ...
        N, o.n_fwd, o.n_bwd, o.n_holdout, out_dir);
end

if isempty(out)
    warning('parse_sweep_log:empty', 'No sweep sections found in %s.', log_file);
end
end
