function [B, tag] = apply_tilt(B, file, tc, baselineFile)
% apply_tilt — the 'TiltCorrect' option of the plot functions.
%
%   [B, tag] = apply_tilt(B, file, tc, baselineFile)
%
% B is the field read from file (N x 3, rows in file order). tc is the option's value:
%   false          B is returned as it is
%   true           B rotated back to the reference orientation with tilt_correct:
%                  the baseline scan's centre point if there is a baseline (so the
%                  scan and its baseline share one reference), else this scan's centre
%   a reference    passed to tilt_correct as it is: 'centre', 'mean', another
%                  scan's file name, or a gravity vector [gx gy gz]
% Points with no usable accelerometer reading come back as NaN (the plots leave
% them out). tag is ', tilt-corrected' when the correction was applied, else ''.

tag = '';
if isempty(tc) || ((islogical(tc) || isnumeric(tc)) && isscalar(tc) && ~tc)
    return
end
if (islogical(tc) || isnumeric(tc)) && isscalar(tc)
    ref = 'centre';
    if nargin > 3 && ~isempty(baselineFile), ref = baselineFile; end
else
    ref = tc;
end

fid = fopen(file);
if fid < 0, error('Cannot open %s', file); end
hdr = strsplit(strtrim(fgetl(fid)), ',');
fclose(fid);
if any(strcmp(hdr, 'Bx_raw_G'))
    error(['%s is already tilt-corrected (a _tiltcorr.csv). Plot it without ' ...
        '''TiltCorrect'', or give the original CSV.'], file);
end

S = tilt_correct(file, 'Reference', ref);
B = S.B_corr_G;
nBad = nnz(~S.ok & all(isfinite(S.B_raw_G), 2));
if nBad
    warning('%d points have no usable accelerometer reading and are left out.', nBad);
end
tag = ', tilt-corrected';
end
