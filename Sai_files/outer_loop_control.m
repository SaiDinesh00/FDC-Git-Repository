function [neta_des, T_des] = outer_loop_control(Xd, pos_ddot)

%% Parameters
g = 9.81;
mass = 12;

%% Variable Unpacking
x_ddot = pos_ddot(1);
y_ddot = pos_ddot(2);
z_ddot = pos_ddot(3);


psi_des = Xd(9);
T_des = mass*sqrt(x_ddot^2 + y_ddot^2 + (g-z_ddot)^2);

phi_des = asin((-mass*x_ddot*sin(psi_des) + mass*y_ddot*cos(psi_des))/T_des);

theta_des = asin((-mass*x_ddot*cos(psi_des) - mass*y_ddot*sin(psi_des))/T_des);

neta_des = [cos(phi_des/2)*cos(theta_des/2)*cos(psi_des/2) + sin(phi_des/2)*sin(theta_des/2)*sin(psi_des/2);
            sin(phi_des/2)*sin(theta_des/2)*cos(psi_des/2) - cos(phi_des/2)*sin(theta_des/2)*sin(psi_des/2);
            cos(phi_des/2)*sin(theta_des/2)*cos(psi_des/2) + sin(phi_des/2)*cos(theta_des/2)*sin(psi_des/2);
            cos(phi_des/2)*cos(theta_des/2)*sin(psi_des/2) - sin(phi_des/2)*sin(theta_des/2)*cos(psi_des/2)];
