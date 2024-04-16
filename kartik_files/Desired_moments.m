function [Md_t, w_d, w_d_dot] = Desired_moments(X, X_dot, Xd, Xd_dot, F_M_wing, dt, flag)
    
    Control_gains;
    
    persistent w_e w_e_dot
    if isempty(w_e)
        w_e = [0; 0; 0];
    else
        w_e = w_e + w_e_dot * dt;
    end

    p_w = X(11);
    q_w = X(12);
    r_w = X(13);

    Quater_d = Xd(4:7, flag);          % It is conjugate
    Quaterd_dot = Xd_dot(4:7);         % It is conjugate
    Quater = X(4:7);
    Quater_dot = X_dot(4:7);
    Quater_e_desired = [1; 0; 0; 0];

    Quater_e = Quaternion_rot_mat(Quater_d, 3) * Quater;
    Quater_e_dot = Quaternion_rot_mat(Quaterd_dot, 3) * Quater + ...
                   Quaternion_rot_mat(Quater_d, 3) * Quater_dot;

    Quater_e_ddot = -2*zeta_quat.*omega_n_quat.*Quater_e_dot ...
                    -omega_n_quat.*omega_n_quat.*(Quater_e - Quater_e_desired);

    w_e_dot = 2 * Quaternion_rot_mat(Quater_e, 2)' * Quater_e_ddot;
    Rot_matrix = Quaternion_rot_mat(Quater_e, 1)';
    w_d = Rot_matrix' * (X(11:13) - w_e);
    if flag == 2
        w_d_dot = (w_d - Xd(11:13, flag)) / dt;
    else
        w_d_dot = [0; 0; 0];
    end

    Rot_matrix_dot = - [0 -r_w q_w; 
                        r_w 0 -p_w;
                        -q_w p_w 0] * Rot_matrix;

    w_dot = w_e_dot + Rot_matrix_dot * w_d + Rot_matrix * w_d_dot;
    Md_t = [Ixx; Iyy; Izz] .* w_dot + ...
            cross(X(11:13), [Ixx; Iyy; Izz] .*X(11:13)) - F_M_wing(2:4);

end