function X_dot = Biplane_6DOF_new(X, U)

% U represents thrust cooefficients
% X represents states in Quadcopter frame
% X = [x, y, z, eta_0, eta_1, eta_2, eta_3, u, v, w,   p,   q,  r]
     %[(1, 2, 3),(4,   5,     6,     7, )(8, 9, 10), (11, 12, 13)]
     
    parameters;
    Quaters    = X(4:7);
    Body_rates = X(11:13);
    F_M_Wing   = [0; 1; 2; 3];
    M_Wing     = F_M_Wing(2:4);
    F_Wing     = F_M_Wing(1);
    F_M_Quad   = control_allocation(U);
    T_Quad     = F_M_Quad(1);
    M_Quad     = F_M_Quad(2:4);

    Vel_dot = [0; 0; 0];
    Pos_dot = [0; 0; 0];

    Att_dot = I_w^-1 * (M_Quad + M_Wing - cross(Body_rates, I_w * Body_rates));
    Quat_rot_mat2 = Quaternion_rot_mat(Quaters, 2);
    Quat_dot = 0.5 * Quat_rot_mat2 * Body_rates;

    X_dot = [Pos_dot;
             Quat_dot;
             Vel_dot;
             Att_dot];
end