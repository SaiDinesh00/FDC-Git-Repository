rho = 1.225;
N_b = 2;
R = 6;
C = 0.4;
Cd0 = 0.01;



CT = 0.5*sigma*Cl_alpha*(theta_0_i/3 - lambda_i/2);
CQ = 0.5*sigma*(lambda_i*Cl_alpha*theta_0_i/3 - lambda_i^2*Cl_alpha/2 + C_d0/4);
sigma = Nb *c / pi*R;