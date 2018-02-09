function [x,y,z] = FK(q0,q1,q2,q3)
    l1 = 119.5;
    l2 = 97.7;
    l3 = 98.03;
    l45 = 115.8;
    dh_table = [q0,l1, 0, pi/2;
    pi/2+q1, 0, l2, 0;
    q2, 0, l3, 0;
    q3, 0,l45, 0;
    ];
    T = eye(4);
    for i  = 1 : length(dh_table)
        theta_i = dh_table(i,1);
        d_i = dh_table(i,2);
        a_i = dh_table(i,3);
        alpha_i = dh_table(i,4);
        T = T * [cos(theta_i),-sin(theta_i)*cos(alpha_i),sin(theta_i)*sin(alpha_i),a_i*cos(theta_i);
            sin(theta_i),cos(theta_i)*cos(alpha_i),-cos(theta_i)*sin(alpha_i),a_i*sin(theta_i);
            0,sin(alpha_i),cos(alpha_i),d_i;0,0,0,1];
    end
    K = T * [0;0;0;1];
    x = K(1);
    y = K(2);
    z = K(3);
end