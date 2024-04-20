function Pos_ddot_inertial = Translational_error_dynamics (X, Xd)
% X = [x,y,z,phi,theta,psi,u,v,w,p,q,r]; State Definition 
  parameters;
  % Inertial frame
    x = X(1);         xd = Xd(1);
    y = X(2);         yd = Xd(2);
    z = X(3);         zd = Xd(3);
% Desired acceleration and velocity are assumed to be zero
   xd_ddot = 0;  xd_dot = 0; %
   yd_ddot = 0;  yd_dot = 0; % 
   zd_ddot = 0;  zd_dot = 0; % 
% Body frame 
    u = X(4);
    v = X(5);
    w = X(6);
% Body frame
%     p = X(10);
%     q = X(11);
%     r = X(12);
% Vehicle, Vehicle 1, Vehicle 2 frames
    n0   = X(7);
    n1 = X(8);
    n2   = X(9);
    n3 = X(10, :);
% Current velocity and position
   Pos_dot_outer =  quat2rotm([n0, n1, n2, n3])*[u; v; w];
   x_dot = Pos_dot_outer (1);
   y_dot = Pos_dot_outer (2);
   z_dot = Pos_dot_outer (3);

% Outer loop design - Translation Dynamics 

% Position_double_dot % Acceleration determination using second order error dynamics 
% Position double dot vector in iertial frame
    
    Pos_ddot_inertial = [ xd_ddot + 2*zitaxt*wnxt*(xd_dot-x_dot) + wnxt^2*(xd-x)
                          yd_ddot + 2*zitaxt*wnxt*(yd_dot-y_dot) + wnxt^2*(yd-y)
                          zd_ddot + 2*zitaxt*wnxt*(zd_dot-z_dot) + wnxt^2*(zd-z) ];        
    

