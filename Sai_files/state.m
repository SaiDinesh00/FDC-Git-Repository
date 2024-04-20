function Xn = state(X, U, dt)

    XX = [X; U];
    [t, XXn] = rk4_m(@state_fun, 0, dt, dt, XX);
    Xn = XXn(1:17, 2);

end
