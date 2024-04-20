function XX_dot = state_fun(t, XX)

    X = XX(1:17, :);
    U = XX(18:21, :);

    X_dot = dynamics2(X, U);

    XX_dot = [X_dot; zeros(4, 1)];  % Assuming the last 4 states are not changing

end
