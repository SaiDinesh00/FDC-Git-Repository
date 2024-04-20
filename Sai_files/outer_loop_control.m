function [T_d, neta_d] = outer_loop_control(Xd, Pos_ddot)

parameters;
           


% Using DI in error dynamics the double derivative of postion is calculated
% in inertial frame

x_ddot = Pos_ddot(1);
y_ddot = Pos_ddot(2);
z_ddot = Pos_ddot(3);

% Outer loop evaluation to determine the values (phi, theta, thrust desired for the inner loop)
     
T_d = m*sqrt(x_ddot^2+y_ddot^2+(z_ddot-g)^2); % total thrust required by the motors
     
% From paper : Conference Paper Design and nonlinear control of an indoor  quadrotor flying robot by Key Lab. of Integrated Autom. of Process Ind., Northeastern Univ., Shenyang, China
% DOI: 10.1109/WCICA.2010.5553974 Conference: Intelligent Control and Automation (WCICA), 2010 8th World Congress on Source: IEEE Xplore
%      ux = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
%      uy = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
      
ux = -m*x_ddot/T_d;
uy = -m*y_ddot/T_d;

psi_d   = Xd(4);

phi_d    = asin(ux*sin(psi_d)-uy*cos(psi_d));
theta_d  = asin((ux*cos(psi_d)+uy*sin(psi_d))/cos(phi_d));

% Solution from outer loop

neta_d = eul2quat([psi_d, theta_d, phi_d],'ZYX');

end 

