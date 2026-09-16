function run_scan(csvfile, port)
% run_scan.m — host-side scan orchestrator.
%
% Streams pulse_table.csv to the Arduino one row at a time, waits for it
% to confirm the move settled, then calls read_magnetometer.m (implement
% that separately for whatever sensor interface you're using — it's not
% wired through the Arduino) and logs the result. Rows are written to
% disk as they complete, so an interrupted scan can be resumed: rerun
% with the same csvfile and it skips indices already in scan_results.csv.
%
% Usage: run_scan('pulse_table.csv', 'COM3')

if nargin < 1, csvfile = 'pulse_table.csv'; end
if nargin < 2, port = 'COM3'; end   % <-- set to your Arduino's actual port

T = readtable(csvfile);
outfile = 'scan_results.csv';

done_idx = [];
if isfile(outfile)
    prev = readtable(outfile);
    done_idx = prev.idx;
    fprintf('Resuming: %d points already done.\n', numel(done_idx));
else
    fid = fopen(outfile, 'w');
    fprintf(fid, 'idx,x,y,z,bx,by,bz\n');
    fclose(fid);
end

s = serialport(port, 115200);
configureTerminator(s, "LF");
flush(s);
pause(2);   % let the Arduino finish resetting after the port opens

writeline(s, "H");
fprintf('Home: %s\n', readline(s));

n = height(T);
for k = 1:n
    if T.valid(k) == 0
        continue
    end
    if ismember(T.idx(k), done_idx)
        continue
    end

    cmd = sprintf('M,%d,%d,%d,%d', T.idx(k), T.pw1(k), T.pw2(k), T.pw3(k));
    writeline(s, cmd);

    ack = readline(s);
    if ~startsWith(ack, "OK")
        warning('Point %d: unexpected response "%s", skipping.', T.idx(k), ack);
        continue
    end

    [bx, by, bz] = read_magnetometer();

    fid = fopen(outfile, 'a');
    fprintf(fid, '%d,%.3f,%.3f,%.3f,%.6f,%.6f,%.6f\n', ...
        T.idx(k), T.x(k), T.y(k), T.z(k), bx, by, bz);
    fclose(fid);

    if mod(k, 50) == 0
        fprintf('%d / %d points done\n', k, n);
    end
end

writeline(s, "H");
readline(s);
clear s

fprintf('Scan complete. Results in %s\n', outfile);
