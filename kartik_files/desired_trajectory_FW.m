
%% Desired trajectories
t = 60 : 0.1: 180;
len_t = length(t);
psi_w_d = zeros(len_t, 1);
X_w_d = zeros(len_t+1, 2);
Xd_w_d = zeros(len_t, 2);
V_w_d = 15;
dt = 0.1;

for i = 1:len_t
if t(i) < 90
    psi_w_d(i) = deg2rad(0);
elseif t(i) >= 90 && t(i) <= 140
    psi_w_d(i) = deg2rad(2 * (t(i) - 90));
else
    psi_w_d(i) = deg2rad(100);
end
Xd_w_d(i, 1) = V_w_d * cos(psi_w_d(i));
Xd_w_d(i, 2) = V_w_d * sin(psi_w_d(i));
X_w_d(i+1, :) = X_w_d(i, :) + Xd_w_d(i, :) * dt;
end

figure(1)
plot(t, rad2deg(psi_w_d))
figure(2)
plot(t, X_w_d(1:len_t, :))