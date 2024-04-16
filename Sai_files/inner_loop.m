function tau_des = inner_loop(X, X_des, neta_des)

I = diag([1.86, 2.031, 3.617]);


omega = X(11:13);
neta = X(7:10);
n0 = neta(1);
n1 = neta(2);
n2 = neta(3);
n3 = neta(4);

neta_dot = 0.5 .* [-n1, -n2, -n3;
                n0, -n3, n2;
                n3, n0, -n1;
                -n2, n1, n0] * omega;

omega_des = X_des(10:12);
neta_des_dot = 0.5*quaternProd(neta_des', [0, omega_des]);
neta_des_dot_conj = quaternConj(neta_des_dot);
neta_des_conj = quaternConj(neta_des');
neta_err_dot = quaternProd(neta_des_dot_conj, neta') + quaternProd(neta_des_conj, neta_dot');
neta_des_conj = quaternConj(neta_des');
neta_err = quaternProd(neta_des_conj, neta');
R_neta_err = quat2rotm(neta_err);
ne0 = neta_err(1);
ne1 = neta_err(2);
ne2 = neta_err(3);
ne3 = neta_err(4);

ned0 = neta_err_dot(1);
ned1 = neta_err_dot(2);
ned2 = neta_err_dot(3);
ned3 = neta_err_dot(4);

G_neta_err = [-ne1, -ne2, -ne3;
                ne0, -ne3, ne2;
                ne3, ne0, -ne1;
                -ne2, ne1, ne0]';

G_dot_neta_err = [-ned1, -ned2, -ned3;
                ned0, -ned3, ned2;
                ned3, ned0, -ned1;
                -ned2, ned1, ned0]'; % TO be Discussed
cap_omega = 2*G_neta_err*G_dot_neta_err';
omega_des_dot = [0; 0; 0]; % TBD
R_dot_neta_err_transpose = -cap_omega*R_neta_err';
R_dot_neta_err = R_dot_neta_err_transpose';
neta_err_des = [1; 0; 0; 0];
zeta_q = diag([0.95, 0.95, 0.95, 0.95]);
nat_freq_q = diag([25, 25, 25, 25]);
omega_dot = (2.*G_neta_err)*(-2.*zeta_q*nat_freq_q'*neta_err_dot' - nat_freq_q*nat_freq_q'*(neta_err-neta_err_des)') + R_dot_neta_err*omega_des' + R_neta_err*omega_des_dot;
tau_des = I*omega_dot + cross(omega, I*omega);
end