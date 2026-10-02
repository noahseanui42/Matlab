function st = delta_status(s, timeout_s)
% delta_status — return the next firmware status line as a struct
%   {"deg":[t1,t2,t3],"mv":0,"run":0,"en":0,"e":0}
% or [] if none arrives within timeout_s. Partial or garbled lines are skipped.

st = [];
t0 = tic;
while toc(t0) < timeout_s
    if s.NumBytesAvailable == 0
        pause(0.005);
        continue
    end
    line = readline(s);
    if isempty(line) || ismissing(line)
        continue
    end
    try
        j = jsondecode(char(line));
    catch
        continue
    end
    if isstruct(j) && all(isfield(j, {'deg', 'mv', 'en', 'e'}))
        st = j;
        return
    end
end
end
