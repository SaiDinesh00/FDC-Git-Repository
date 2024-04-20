parameters;
Pos_wing     = [0; 0; -200];
Quaters_wing = [1; 0; 1; 0] / norm([1; 0; 1; 0]);
Vel_wing     = [0; 0; 0];
Att_wing     = [0.2; 0.4; 0.7];

Pos_wing_des     = [0; 0; -200];
Quaters_wing_des = [1; 0; 0; 0] / norm([1; 0; 0; 0]);
Vel_wing_des     = [0; 0; 0];
Att_wing_des     = [0; 0; 0];

X = zeros(13, N);
Xd= zeros(13, N);
X_dot = zeros(13, N);
U = zeros(4, N);

F_M_Quad_des = zeros(4, N);


X(:, 1) = [Pos_wing;
           Quaters_wing;
           Vel_wing;
           Att_wing];

Xd(:, 1) = [Pos_wing_des;
           Quaters_wing_des;
           Vel_wing_des;
           Att_wing_des];

U(:, 1) = CT_init * ones(4, 1);

Xd(4, :) = ones(1, N);