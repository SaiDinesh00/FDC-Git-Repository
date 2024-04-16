function [Q, Euler_angles_d] = Inner_loop(X, Xd, X_dot, Xd_dot, psi_d, F_M_wing)

    u_w = X(8);
    v_w = X(9);
    w_w = X(10);
    p = X(11);
    q = X(12);
    r = X(13);
    h = X(3);
    hd = Xd(3);
    hd_dot = Xd_dot(3);
    quaters = X(4:7)';
    Euler_angles = quat2eul(quaters);
    phi = Euler_angles(1);
    theta = Euler_angles(2);
    psi = Euler_angles(3);
    V = norm(X(8:10));
    V_dot = norm(X_dot(8:10));
    if v_w == 0
        beta = 0;
    else
        beta = asin(v_w/V);
    end
    Fay = F_M_wing(2);

    Control_gains;

    ah = u_w;
    bh =  v_w * sin(phi) + w_w * cos(phi); 
    
    %% Desired pitching angle
    if (sqrt(ah^2 + bh^2)) == 0
        theta_d = 0;
    else
        Ah = (hd_dot - kh * (h - hd)) / (sqrt(ah^2 + bh^2));
        Bh = atan2(bh, ah);
        if Ah > 1
            theta_d = 0;
        else
            if norm(Ah) <= 1
                theta_d = asin(Ah) + Bh;
            else
                theta_d = Bh;
            end
        end
    end

    %% Desired rolling angle
    Num = r * u_w - p * w_w - (Fay/m) + V_dot * sin(beta) - ...
          kb * beta * V * cos(beta);
    Den = g * cos(theta);
    if norm((Num/Den)) > 1
        phi_d = 0;
    else
        phi_d = asin(Num/Den);
    end

    %% Desired Quaterions
    Euler_angles_d = [phi_d, theta_d, psi_d];
    Q = eul2quat(Euler_angles_d);

end