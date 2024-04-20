function [Ct_dot, T, L_tbar, M_t, N_t] = control_allocation(X, T_des, tau_des)
    
parameters

ct1 = X(14);
ct2 = X(15);
ct3 = X(16);
ct4 = X(17);

kt = 125;
kl = 125;
km = 125;
kn = 1000;
T = K*(ct1 + ct2 + ct3 + ct4);
L_tbar = K * l *(ct1 - ct2 - ct3 + ct4);
M_t = K * l * (ct1 + ct2 - ct3 - ct4);
N_t = (K*R/sqrt(2)) * (abs(ct1)^1.5 - abs(ct2)^1.5 + abs(ct3)^1.5 - abs(ct4)^1.5);

L_t_bar_des = tau_des(1);
M_t_des = tau_des(2);
N_t_des = tau_des(3);

thrust_tau_dot_err = [kt*(T_des - T);
                      kl*(L_t_bar_des - L_tbar);
                      km*(M_t_des - M_t);
                      kn*(N_t_des - N_t)];
mixing_matrix = [K K K K;
                 K*l -K*l -K*l K*l;
                 K*l K*l -K*l -K*l;
                 (3*K*R/2)*sqrt(abs(ct1)/2) -(3*K*R/2)*sqrt(abs(ct2)/2) (3*K*R/2)*sqrt(abs(ct3)/2) -(3*K*R/2)*sqrt(abs(ct4)/2)];
Ct_dot = pinv(mixing_matrix)*thrust_tau_dot_err;

end