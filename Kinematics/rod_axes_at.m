function nhat = rod_axes_at(p, sb, sp, L1, L2)
% Nominal ball-stud axes: rod unit vectors at a reference point.
th = ik_batch(p, sb, sp, L1, L2);

wb = sb/(2*sqrt(3));
up = sp/sqrt(3);
wp = sp/(2*sqrt(3));
s3 = sqrt(3)/2;

B    = [0, -wb, 0;  s3*wb, wb/2, 0;  -s3*wb, wb/2, 0];
e    = [0, -1;  s3, 0.5;  -s3, 0.5];
Prel = [0, -up, 0;  sp/2, wp, 0;  -sp/2, wp, 0];

nhat = zeros(3,3);
for i = 1:3
    thr = th(i)*pi/180;
    A = B(i,:) + L1*[cos(thr)*e(i,1), cos(thr)*e(i,2), -sin(thr)];
    rod = (p + Prel(i,:)) - A;
    nhat(i,:) = rod/norm(rod);
end