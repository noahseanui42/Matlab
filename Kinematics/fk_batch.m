function p = fk_batch(th_deg, sb, sp, L1, L2)
% Forward kinematics for the delta robot: given per-arm bicep angles,
% recover the platform reference point by trilaterating the three
% forearm-length spheres. Inverse of the geometry in ik_batch.m /
% validate_batch.m (same B, e, Prel construction).
%#codegen
wb = sb/(2*sqrt(3));
up = sp/sqrt(3);
wp = sp/(2*sqrt(3));
s3 = sqrt(3)/2;

B    = [0, -wb, 0;  s3*wb, wb/2, 0;  -s3*wb, wb/2, 0];
e    = [0, -1;  s3, 0.5;  -s3, 0.5];
Prel = [0, -up, 0;  sp/2, wp, 0;  -sp/2, wp, 0];

N = size(th_deg,1);
p = zeros(N,3);

for k = 1:N
    C = zeros(3,3);
    for i = 1:3
        thr = th_deg(k,i)*pi/180;
        A = B(i,:) + L1*[cos(thr)*e(i,1), cos(thr)*e(i,2), -sin(thr)];
        C(i,:) = A - Prel(i,:);
    end

    % Trilateration: solve for the point at distance L2 from all three
    % sphere centres C(1,:), C(2,:), C(3,:).
    ex = (C(2,:) - C(1,:)) / norm(C(2,:) - C(1,:));
    d  = norm(C(2,:) - C(1,:));
    ii = dot(ex, C(3,:) - C(1,:));
    eyRaw = C(3,:) - C(1,:) - ii*ex;
    ey = eyRaw / norm(eyRaw);
    ez = cross(ex, ey);
    jj = dot(ey, C(3,:) - C(1,:));

    x = d/2;
    y = (ii^2 + jj^2)/(2*jj) - (ii/jj)*x;
    z2 = L2^2 - x^2 - y^2;
    z = -sqrt(max(z2, 0));   % platform sits below the base plane; flip sign if this doesn't match

    p(k,:) = C(1,:) + x*ex + y*ey + z*ez;
end
