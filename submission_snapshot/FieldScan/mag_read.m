function [Bmean, Bstd] = mag_read(mag, n, dt_s)
% mag_read — average n magnetometer samples taken dt_s apart.
% Returns 1x3 mean and standard deviation in gauss, sensor axes.
% Samples the sensor rejects (e.g. out of range) are dropped.

B = nan(n, 3);
for k = 1:n
    try
        B(k,:) = cellfun(@double, cell(mag.getMagneticField()));
    catch
        % leave this sample as NaN
    end
    pause(dt_s);
end
Bmean = mean(B, 1, 'omitnan');
Bstd  = std(B, 0, 1, 'omitnan');
end
