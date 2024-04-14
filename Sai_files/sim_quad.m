clc, close all
clear

Initial_state = zeros(13,1);
Initial_state(7:10) = eul2quat([10, 0, 0],"ZYX");
X(:,1) = Initial_state;
dt = 1e-2;

for i = 1:1e3-1
    u(:, i) = ones(4,1)*1e3;
    X(:, i+1) = X(:, i) + dt .* dynamics2(X(:, i), u(:, i));
end
plot(X(3,:))