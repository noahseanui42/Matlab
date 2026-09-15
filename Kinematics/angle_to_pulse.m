function [pw, sat] = angle_to_pulse(th_deg, k_us, pw_home, sgn, pw_min, pw_max)
%#codegen
N = size(th_deg,1);
pw  = zeros(N,3);
sat = false(N,3);

for k = 1:N
    for i = 1:3
        if isnan(th_deg(k,i))
            pw(k,i)  = pw_home(i);     % park invalid points at home
            sat(k,i) = true;
            continue
        end
        p = round(pw_home(i) + sgn(i)*k_us(i)*th_deg(k,i));
        if p < pw_min(i)
            p = pw_min(i);  sat(k,i) = true;
        elseif p > pw_max(i)
            p = pw_max(i);  sat(k,i) = true;
        end
        pw(k,i) = p;
    end
end