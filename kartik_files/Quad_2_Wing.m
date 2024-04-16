function X_rot = Quad_2_Wing(X)

rot_mat = [0 0 -1;
           0 1 0; 
           1 0 0];

X_rot = zeros(17, 1);

%% States rotation
    % Inertial frame
    X_rot(1:3) = rot_mat * X(1:3);
    
    % Quaternions Vehicle, Vehicle 1, Vehicle 2 frames
    Euler_angles = quat2eul(X(4:7)',"ZYX");
    Euler_angles_rot = rot_mat * Euler_angles';
    X_rot(4:7) = eul2quat(Euler_angles_rot'); % [w=e0, x=e1, y=e2, z=e3]
    
    % Linear velocities in Body frame 
    X_rot(8:10) = rot_mat * X(8:10);
    
    % Angular velocities in Body frame
    X_rot(11:13) = rot_mat * X(11:13);

end