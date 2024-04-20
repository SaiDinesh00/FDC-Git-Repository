
%% X, Y, and Z plots

% figure
% subplot(3,1,1);
% ax1 = gca;
% plot(t, X(1,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, Xd(1,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% ylabel("X (m)")
% ax1.LineWidth = 1.2;
% ax1.FontSize = 12;
% ax1.FontWeight = "bold";
% legend;
% 
% subplot(3,1,2); 
% ax2 = gca;
% plot(t, X(2,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, Xd(2,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% ylabel("Y (m)")
% ax2.LineWidth = 1.2;
% ax2.FontSize = 12;
% ax2.FontWeight = "bold";
% 
% subplot(3,1,3); 
% ax3 = gca;
% plot(t, X(3,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, Xd(3,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% xlabel("Time (seconds)")
% ylabel("Z (m)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";


%% Velocities in body frame

% figure
% subplot(2, 2, 1);
% ax1 = gca;
% plot(t, X(8,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, Xd(8,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% xlabel("Time (seconds)")
% ylabel("u_w (m/s)")
% ax1.LineWidth = 1.2;
% ax1.FontSize = 12;
% ax1.FontWeight = "bold";
% legend;
% 
% subplot(2, 2, 2); 
% ax2 = gca;
% plot(t, X(9,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, Xd(9,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% xlabel("Time (seconds)")
% ylabel("v_w (m/s)")
% ax2.LineWidth = 1.2;
% ax2.FontSize = 12;
% ax2.FontWeight = "bold";
% 
% subplot(2, 2, 3); 
% ax3 = gca;
% plot(t, X(10,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, Xd(10,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% xlabel("Time (seconds)")
% ylabel("w_w (m/s)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";
% 
% subplot(2, 2, 4); 
% ax3 = gca;
% plot(t, vecnorm(X(8:10,:)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% hold on
% plot(t, vecnorm(Xd(8:10,:)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
% xlabel("Time (seconds)")
% ylabel("V (m/s)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";


%% Quaternions

figure
subplot(2, 2, 1);
ax1 = gca;
plot(t, X(4,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, Xd(4,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
xlabel("Time (seconds)")
ylabel("\eta_{0_w}")
ax1.LineWidth = 1.2;
ax1.FontSize = 12;
ax1.FontWeight = "bold";
legend;

subplot(2, 2, 2); 
ax2 = gca;
plot(t, X(5,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, Xd(5,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
xlabel("Time (seconds)")
ylabel("\eta_{1_w}")
ax2.LineWidth = 1.2;
ax2.FontSize = 12;
ax2.FontWeight = "bold";

subplot(2, 2, 3); 
ax3 = gca;
plot(t, X(6,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, Xd(6,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
xlabel("Time (seconds)")
ylabel("\eta_{2_w}")
ax3.LineWidth = 1.2;
ax3.FontSize = 12;
ax3.FontWeight = "bold";

subplot(2, 2, 4); 
ax3 = gca;
plot(t, Xd(7,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, Xd(7,:),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
xlabel("Time (seconds)")
ylabel("\eta_{3_w}")
ax3.LineWidth = 1.2;
ax3.FontSize = 12;
ax3.FontWeight = "bold";

%% Euler rates

Euler_angles = quat2eul(X(4:7, :)');
figure
subplot(3,1,1);
ax1 = gca;
plot(t, rad2deg(Euler_angles(:, 1)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
% plot(t(1:N-1), rad2deg(Euler_angles_des(1, :)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
ylabel("\phi_w (deg)")
ax1.LineWidth = 1.2;
ax1.FontSize = 12;
ax1.FontWeight = "bold";
legend;

subplot(3,1,2); 
ax2 = gca;
plot(t, rad2deg(Euler_angles(:, 2)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
% plot(t(1:N-1), rad2deg(Euler_angles_des(2, :)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
ylabel("\theta_w (deg)")

ax2.LineWidth = 1.2;
ax2.FontSize = 12;
ax2.FontWeight = "bold";

subplot(3,1,3); 
ax3 = gca;
plot(t, rad2deg(Euler_angles(:, 3)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
% plot(t(1:N-1), rad2deg(Euler_angles_des(3, :)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
% hold off
ylabel("\psi_w (deg)")
xlabel("Time (seconds)")
ax3.LineWidth = 1.2;
ax3.FontSize = 12;
ax3.FontWeight = "bold";

%% Body rates p, q, r

figure
subplot(3,1,1);
ax1 = gca;
plot(t, rad2deg(X(11,:)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, rad2deg(Xd(11,:)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
ylabel("p (^\circ/sec)")
ax1.LineWidth = 1.2;
ax1.FontSize = 12;
ax1.FontWeight = "bold";
legend;

subplot(3,1,2); 
ax2 = gca;
plot(t, rad2deg(X(12,:)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, rad2deg(Xd(12,:)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
ylabel("q (^\circ/sec)")
ax2.LineWidth = 1.2;
ax2.FontSize = 12;
ax2.FontWeight = "bold";

subplot(3,1,3); 
ax3 = gca;
plot(t, rad2deg(X(13,:)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
hold on
plot(t, rad2deg(Xd(13,:)),'--', 'LineWidth', 3, 'Color','m', 'DisplayName', 'Desired')
hold off
xlabel("Time (seconds)")
ylabel("r (^\circ/sec)")
ax3.LineWidth = 1.2;
ax3.FontSize = 12;
ax3.FontWeight = "bold";

%% Forces and moments

% figure
% subplot(2, 2, 1);
% ax1 = gca;
% plot(t(1:N-1), F_M_quad(1,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("T_t (N)")
% ax1.LineWidth = 1.2;
% ax1.FontSize = 12;
% ax1.FontWeight = "bold";
% % legend;
% 
% subplot(2, 2, 2); 
% ax2 = gca;
% plot(t(1:N-1), F_M_quad(2,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("L_t (N-m)")
% ax2.LineWidth = 1.2;
% ax2.FontSize = 12;
% ax2.FontWeight = "bold";
% 
% subplot(2, 2, 3); 
% ax3 = gca;
% plot(t(1:N-1), F_M_quad(3,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("M_t (N-m)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";
% 
% subplot(2, 2, 4); 
% ax3 = gca;
% plot(t(1:N-1), F_M_quad(4,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("N_t (N-m)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";


% figure
% subplot(2, 3, 1);
% ax1 = gca;
% plot(t(1:N-1), F_M_wing(1,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("F_{xw} (N)")
% ax1.LineWidth = 1.2;
% ax1.FontSize = 12;
% ax1.FontWeight = "bold";
% % legend;
% 
% subplot(2, 3, 2); 
% ax2 = gca;
% plot(t(1:N-1), F_M_wing(2,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("F_{yw} (N)")
% ax2.LineWidth = 1.2;
% ax2.FontSize = 12;
% ax2.FontWeight = "bold";
% 
% subplot(2, 3, 3); 
% ax3 = gca;
% plot(t(1:N-1), F_M_wing(3,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("F_{zw} (N)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";
% 
% subplot(2, 3, 4); 
% ax3 = gca;
% plot(t(1:N-1), F_M_wing(4,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("L_w (N-m)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";
% 
% subplot(2, 3, 5); 
% ax3 = gca;
% plot(t(1:N-1), F_M_wing(5,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("M_w (N-m)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";
% 
% subplot(2, 3, 6); 
% ax3 = gca;
% plot(t(1:N-1), F_M_wing(6,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% xlabel("Time (seconds)")
% ylabel("N_w (N-m)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";    

%% Thrust coefficients

figure
subplot(2, 2, 1);
ax1 = gca;
plot(t, U(1,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
xlabel("Time (seconds)")
ylabel("C_{T_1}")
ax1.LineWidth = 1.2;
ax1.FontSize = 12;
ax1.FontWeight = "bold";
% legend;

subplot(2, 2, 2); 
ax2 = gca;
plot(t, U(2,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
xlabel("Time (seconds)")
ylabel("C_{T_2}")
ax2.LineWidth = 1.2;
ax2.FontSize = 12;
ax2.FontWeight = "bold";

subplot(2, 2, 3); 
ax3 = gca;
plot(t, U(3,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
xlabel("Time (seconds)")
ylabel("C_{T_3}")
ax3.LineWidth = 1.2;
ax3.FontSize = 12;
ax3.FontWeight = "bold";

subplot(2, 2, 4); 
ax3 = gca;
plot(t, U(4,:), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
xlabel("Time (seconds)")
ylabel("C_{T_4}")
ax3.LineWidth = 1.2;
ax3.FontSize = 12;
ax3.FontWeight = "bold";

%% Angle of attack

% figure
% ax1 = gca;
% plot(t, atan2d(X(10, :), X(8, :)), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% ylabel("\alpha (deg)")
% ax1.LineWidth = 1.2;
% ax1.FontSize = 12;
% ax1.FontWeight = "bold";

%% Aero forces

% figure
% subplot(3,1,1);
% ax1 = gca;
% plot(t(1:N-1), Aero_force(1, :), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% ylabel("Lift (N)")
% ax1.LineWidth = 1.2;
% ax1.FontSize = 12;
% ax1.FontWeight = "bold";
% legend;
% 
% subplot(3,1,2); 
% ax2 = gca;
% plot(t(1:N-1), Aero_force(2, :), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% ylabel("Drag (N)")
% ax2.LineWidth = 1.2;
% ax2.FontSize = 12;
% ax2.FontWeight = "bold";
% 
% subplot(3,1,3); 
% ax3 = gca;
% plot(t(1:N-1), Aero_force(3, :), 'LineWidth', 3, 'Color','g', 'DisplayName', 'Actual')
% ylabel("Side Force (N)")
% xlabel("Time (seconds)")
% ax3.LineWidth = 1.2;
% ax3.FontSize = 12;
% ax3.FontWeight = "bold";