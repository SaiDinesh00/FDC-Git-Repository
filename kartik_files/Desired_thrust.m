function [T, ud_w] = Desired_thrust(X, Xd_dot, F_M_wing)

Control_gains;
parameters;

e0 = X(4);
e1 = X(5);
e2 = X(6);
e3 = X(7);
Euler_angles = quat2eul(X(4:7)');
theta = Euler_angles(2);

xd_dot = Xd_dot(1);
yd_dot = Xd_dot(2);
zd_dot = Xd_dot(3);

u_w = X(8);
w_w = X(10);
q_w = X(12);
Fax_w = F_M_wing(1);

rot_mat = Quaternion_rot_mat(X(4:7), 1);
ud_w = rot_mat(:, 1)' * Xd_dot(1:3);

ud_dot_w = Xd_dot(8);

sigma_t = (1 / (K_u * ud_w)) * (ud_dot_w + K_u * (ud_w - u_w) + q_w * w_w + ...
           g * sin(theta) - (Fax_w / m));

T = sigma_t * T_max;

end