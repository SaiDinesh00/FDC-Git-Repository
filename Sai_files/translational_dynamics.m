function pos_ddot = translational_dynamics(X, Xd)

    parameters
    %% State Unpacking
    x = X(1);
    y = X(2);
    z = X(3);

    u = X(4);
    v = X(5);
    w = X(6);

    xd = Xd(1);
    yd = Xd(2);
    zd = Xd(3);

    n0 = X(7);
    n1 = X(8);
    n2 = X(9);
    n3 = X(10);

    %% Gains
    

    Rot_mat_body2iner_inquat = quat2rotm([n0, n1, n2, n3]);


    pos_dot = Rot_mat_body2iner_inquat' * [u; v; w];

    pos = [x; y; z];

    pos_des = [xd; yd; zd];
   

    % Assuming zero desired acceleration and velocity
    pos_des_ddot = [0; 0; 0];
    pos_des_dot = [0; 0; 0];

    pos_ddot = pos_des_ddot + 2 * zeta_x * omega_n_x' * (pos_des_dot - pos_dot) + (omega_n_x*omega_n_x') * (pos_des - pos);
    
    
end
