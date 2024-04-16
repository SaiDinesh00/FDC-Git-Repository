function F_M_aero = Aerodynamic_forces_n_moments(X)
    
    parameters;
    u = X(8);
    v = X(9);
    w = X(10);        % Minus or plus sign?
    p = X(11);
    q = X(12);
    r = X(13);
    V = norm(X(8:10));
    if V == 0
        F_M_aero = [0; 0; 0; 0; 0; 0];
        return
    end
    alp = atan2(w, u);
    if v == 0
        beta = 0;
    else
        beta = asin(v/V);
    end

    %% Lift calculation
    CL = Lift_coefficient(CL_0, CL_alp, alp, alp_0, M_0) + ...
        CL_q * (q * c_mac / (2 * V));
    L = 0.5 * rho * V^2 * S_w * CL;

    %% Drag calculation
    k = 1 / (pi * AR * 0.8);
    CD = CD_0 + k * CL^2;
    D = 0.5 * rho * V^2 * S_w * CD;

    %% Side force calculation
    CY = CY_beta * beta + CY_p * (p * b_w / (2 * V)) +...
         CY_r * (r * b_w / (2 * V));
    Y = 0.5 * rho * V^2 * S_w * CY;

    %% Rolling moment calculation
    Cl = Cl_beta * beta + Cl_p * (p * b_w / (2 * V)) +...
         Cl_r * (r * b_w / (2 * V));
    L_a = 0.5 * rho * V^2 * S_w * b_w * Cl;

    %% pitching moment calculation
    Cm = Cm_0 + Cm_alp * alp + Cm_q * (q * c_mac / (2 * V));
    M_a = 0.5 * rho * V^2 * S_w * c_mac * Cm;

    %% Yawing moment calculation
    Cn = Cn_beta * beta + Cn_p * (p * b_w / (2 * V)) +...
         Cn_r * (r * b_w / (2 * V));
    N_a = 0.5 * rho * V^2 * S_w * b_w * Cn;
    
    Rot_mat_Wind2Quad = [sin(alp)*cos(beta) -sin(alp)*sin(beta) cos(alp);
               sin(beta) cos(beta) 0;
               -cos(alp)*cos(beta) cos(alp)*sin(beta) sin(alp)];
    Rot_mat_Quad2Wing = [0 0 -1;
                         0 1 0; 
                         1 0 0];
    Fs = Rot_mat_Quad2Wing * Rot_mat_Wind2Quad * [-D; Y; -L];
    F_M_aero = [Fs; L_a; M_a; N_a];

end