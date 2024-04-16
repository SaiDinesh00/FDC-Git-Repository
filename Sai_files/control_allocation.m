function Ct_dot = control_allocation(X, T_des, tau_des)
    
%% Parameters
mass = 12; % Kg
g = 9.81;
R      = 0.18;
l      = 0.3;
K      = 820;

ct1 = X(14);
ct2 = X(15);
ct3 = X(16);
ct4 = X(17);

kt = 125;
kl = 125;
km = 125;
kn = 125;
T = K*(ct1 + ct2 + ct3 + ct4);
L_tbar = K * l *(ct1 - ct2 - ct3 + ct4);
M_t = K * l * (ct1 + ct2 - ct3 - ct4);
N_t = (K*R/sqrt(2)) * (ct1^1.5 - ct2^1.5 + ct3^1.5 - ct4^1.5);

L_t_bar_des = tau_des(1);
M_t_des = tau_des(2);
N_t_des = tau_des(3);

thrust_tau_dot_err = [kt*(T_des - T);
                      kl*(L_t_bar_des - L_tbar);
                      km*(M_t_des - M_t);
                      kn*(N_t_des - N_t)];
Ct_dot = [K K K K;
          -(3*K*R/2)*sqrt(ct1/2) (3*K*R/2)*sqrt(ct2/2) -(3*K*R/2)*sqrt(ct3/2) (3*K*R/2)*sqrt(ct4/2);
          K*l K*l -K*l -K*l;
          K*l -K*l -K*l K*l;]*thrust_tau_dot_err;

end