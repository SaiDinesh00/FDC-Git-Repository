function  rot_mat_B2E = Quaternion_rot_mat(quats, n)

    e0 = quats(1);
    e1 = quats(2);
    e2 = quats(3);
    e3 = quats(4);
    
    switch n
        case 1
        rot_mat_B2E = [e0^2+e1^2-e2^2-e3^2, 2*e1*e2-2*e3*e0, 2*e1*e3+2*e2*e0;
                   2*e1*e2+2*e3*e0, e0^2-e1^2+e2^2-e3^2, 2*e3*e2-2*e1*e0;
                   2*e1*e3-2*e2*e0, 2*e3*e2+2*e1*e0, e0^2-e1^2-e2^2+e3^2];

        case 2
            rot_mat_B2E = [-e1 -e2 -e3;
                           +e0 -e3 +e2;
                           +e3 +e0 -e1;
                           -e2 +e1 +e0];
        
        case 3
            rot_mat_B2E = [+e0, +e1, +e2, +e3;
                           -e1, +e0, +e3, -e2;
                           -e2, -e3, +e0, +e1;
                           -e3, +e2, -e1, +e0];
    end
end