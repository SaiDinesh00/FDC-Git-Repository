function [psi_w_d, X_w_d, Xd_w_d] = Desired_trajectory(t, X_w_d_1, dt, V_w_d)

if t < 90
    psi_w_d = deg2rad(0);
elseif t >= 90 && t <= 140
    psi_w_d = deg2rad(2 * (t - 90));
else
    psi_w_d = deg2rad(100);
end
Xd_w_d = zeros(3, 1);
Xd_w_d(1) = V_w_d * cos(psi_w_d);
Xd_w_d(2) = V_w_d * sin(psi_w_d);
X_w_d = X_w_d_1 + Xd_w_d * dt;

end