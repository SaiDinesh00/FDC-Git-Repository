function [X_new, X_dot] = state(X, U, F_M_wing, dt)

% X_rotated = Wing_2_Quad(X);
X_new = rk4_m('Biplane_6DOF', dt, X, U, F_M_wing);
X_dot = Biplane_6DOF(X, U, F_M_wing);
end


