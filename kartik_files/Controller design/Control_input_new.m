function U = Control_input_new(F_M_Quad_des, CTs, dt)
    
    Control_gains;
    F_M_Quad = control_allocation(CTs);                     % Wing frame
    Moments_dot = [0;%[K_T * (F_M_Quad_des(1) - F_M_Quad(1));
                   K_L * (F_M_Quad_des(2) - F_M_Quad(2));
                   K_M * (F_M_Quad_des(3) - F_M_Quad(3));
                   K_N * (F_M_Quad_des(4) - F_M_Quad(4))];
    control_mat = Control_matrix(CTs);
    CT_dot = control_mat^-1 * Moments_dot;
    U = CTs + CT_dot * dt;                            % [CT(1:4)]
    Thrust_coeffs = U;
    Thrust_coeffs(Thrust_coeffs > 0.6) = 0.6;
    Thrust_coeffs(Thrust_coeffs < 0.000001) = 0.000001;
    U = Thrust_coeffs;
end