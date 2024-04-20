function Moments_t_des = inner_loop(X, neta_des)

    parameters;
    % Extract quaternions and angular velocity from state X
    neta = X(7:10)';
    omega = X(11:13)';

    % Normalize the desired quaternion
    neta_d = neta_des' / norm(neta_des);

    % Calculate the error quaternion
    neta_d_conj = conj_q(neta_d);
    neta_e = quaternProd(neta_d_conj, neta);
    neta_e = neta_e / norm(neta_e);

    % Calculate the derivative of the error quaternion
    neta_e_dot = 0.5 * quaternProd(neta_e, [0, omega]);
    % neta_err_dot = 0.5 * quaternProd([-neta_e(2:4), neta_e(1)], neta_e_dot);

    % Desired error quaternion for stability
    neta_err_des = [1; 0; 0; 0];

    % Calculate the second derivative of the error quaternion
    neta_err_ddot = -(2 * zeta_quat * omega_n_quat' * neta_e_dot') - (omega_n_quat * omega_n_quat' * (neta_e' - neta_err_des));
    neta_err_ddot = neta_err_ddot / norm(neta_err_ddot);

    % Calculate the G matrix
    G = calculate_G(neta_e);

    % Calculate the desired angular acceleration in the error frame
    omega_e_dot = 2 * G * neta_err_ddot;

    % Calculate the desired moments
    

    CAP_omega = 2*calculate_G(neta_e)*calculate_G(neta_e_dot)';

    Rot_mat = quat2rotm(neta_e);

    Rot_mat_dot_trans = -CAP_omega * Rot_mat';
    

    temp = quaternProd(neta_e, [0, omega]);

    omega_d = quaternProd(temp, quaternConj(neta_e));
    
    omega_d_dot = quat2rotm(neta_d')'*omega_e_dot;
    
    omega_dot = omega_e_dot + Rot_mat_dot_trans*omega_d(2:4)' + quat2rotm(neta_e)'*omega_d_dot;
    

    Moments_t_des = I * omega_dot + cross(omega', I * omega');

    % Display the error quaternion for debugging
    disp('Error Quaternion:');
    disp(neta_e);
end
