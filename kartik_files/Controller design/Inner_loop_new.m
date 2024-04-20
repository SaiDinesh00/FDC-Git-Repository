function Moments_t_des = Inner_loop_new(X, Xd, M_wing)

% X(4:7) = Quaternions
Quater_d        = Xd(4:7);
Quaters         = X(4:7);
Body_rates      = X(11:13);

Control_gains;

Quater_e = Quaternion_rot_mat(Quater_d, 3) * Quaters;
Quater_e_conj = [Quater_e(1); -1 * Quater_e(2:4)];
Quater_e_dot = 0.5 * Quaternion_rot_mat(Quater_e_conj, 3) * [0; Body_rates];
Quater_e_ddot = -2*zeta_quat.*omega_n_quat.*Quater_e_dot ...
            -omega_n_quat.*omega_n_quat.*(Quater_e - Quater_d);
w_e_dot_c = 2 * Quaternion_rot_mat(Quater_e, 2)' * Quater_e_ddot;
Moments_t_des = I_w * w_e_dot_c + cross(Body_rates, I_w * Body_rates) - M_wing;
end