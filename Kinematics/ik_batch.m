function [th_deg, disc] = ik_batch(xyz, sb, sp, L1, L2)
%#codegen
N = size(xyz,1);
th_deg = zeros(N,3);
disc   = zeros(N,3);

wb = sb/(2*sqrt(3));
up = sp/sqrt(3);
wp = sp/(2*sqrt(3));

a = wb - up;
b = sp/2 - (sqrt(3)/2)*wb;
c = wp - wb/2;

for k = 1:N
    x = xyz(k,1);  y = xyz(k,2);  z = xyz(k,3);
    r2 = x*x + y*y + z*z;

    E = zeros(1,3);  F = zeros(1,3);  G = zeros(1,3);
    E(1) = 2*L1*(y + a);
    F(1) = 2*z*L1;
    G(1) = r2 + a*a + L1*L1 + 2*y*a - L2*L2;

    E(2) = -L1*(sqrt(3)*(x + b) + y + c);
    F(2) = 2*z*L1;
    G(2) = r2 + b*b + c*c + L1*L1 + 2*(x*b + y*c) - L2*L2;

    E(3) = L1*(sqrt(3)*(x - b) - y - c);
    F(3) = 2*z*L1;
    G(3) = r2 + b*b + c*c + L1*L1 + 2*(-x*b + y*c) - L2*L2;

    for i = 1:3
        d = E(i)*E(i) + F(i)*F(i) - G(i)*G(i);
        disc(k,i) = d;
        if d < 0
            th_deg(k,i) = NaN;          % unreachable, flagged downstream
        else
            t = (-F(i) - sqrt(d)) / (G(i) - E(i));
            th_deg(k,i) = 2*atan(t)*180/pi;
        end
    end
end