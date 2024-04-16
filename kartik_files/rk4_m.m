function X_f = rk4_m(fun, dt, X, U, F_M_wing)
    
    K1 = feval(fun, X, U, F_M_wing);
    K2 = feval(fun, X + (dt/2)*K1, U, F_M_wing);
    K3 = feval(fun, X + (dt/2)*K2, U , F_M_wing);
    K4 = feval(fun, X + dt*K3, U, F_M_wing);
   
    X_f = X + dt*(K1/6 + K2/3 + K3/3 + K4/6);
end