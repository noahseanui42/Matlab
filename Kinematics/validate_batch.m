function [valid, code, artic_deg, rodres] = validate_batch( ...
        xyz, th_deg, sb, sp, L1, L2, nhat, th_lim, artic_max, rod_tol)
%#codegen
N = size(xyz,1);
valid     = false(N,1);
code      = zeros(N,1,'uint8');
artic_deg = zeros(N,3);
rodres    = zeros(N,3);

wb = sb/(2*sqrt(3));
up = sp/sqrt(3);
wp = sp/(2*sqrt(3));
s3 = sqrt(3)/2;

B = [0, -wb, 0;  s3*wb, wb/2, 0;  -s3*wb, wb/2, 0];
e = [0, -1;  s3, 0.5;  -s3, 0.5];
Prel = [0, -up, 0;  sp/2, wp, 0;  -sp/2, wp, 0];

for k = 1:N
    p = xyz(k,:);
    cbits = uint8(0);

    for i = 1:3
        th = th_deg(k,i);

        if isnan(th)
            cbits = bitor(cbits, uint8(1));
            artic_deg(k,i) = NaN;
            rodres(k,i)    = NaN;
            continue
        end

        if th < th_lim(1) || th > th_lim(2)
            cbits = bitor(cbits, uint8(2));
        end

        thr = th*pi/180;
        A = B(i,:) + L1*[cos(thr)*e(i,1), cos(thr)*e(i,2), -sin(thr)];
        rod = (p + Prel(i,:)) - A;

        Lr = norm(rod);
        rodres(k,i) = abs(Lr - L2);
        if rodres(k,i) > rod_tol
            cbits = bitor(cbits, uint8(8));
        end

        ca = dot(rod/Lr, nhat(i,:));
        ca = min(1, max(-1, ca));              % guard acos domain
        artic_deg(k,i) = acos(ca)*180/pi;
        if artic_deg(k,i) > artic_max
            cbits = bitor(cbits, uint8(4));
        end
    end

    code(k)  = cbits;
    valid(k) = (cbits == 0);
end