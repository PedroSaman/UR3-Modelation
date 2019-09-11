function [Tau] = Function_Model(q,d_q,d_d_q,omega,d_omega,f7,t7,theta)
    
    %theta = [r0_c1;r1_c2;r2_c3;r3_c4;r4_c5;r6_c5;r0_1;r1_2;r2_3;r3_4;r4_5;r5_6
    %         I1;I2;I3;I4;I5;I6;mass]
    %All the 'r'are vectors (3x1), I1 to I6 are Inertia Matrix (6x6), mass is a vector (1X6)
    %f7 and t7 are end-effector relative and both are vectors (3X1)
    
    %Distance parameters
    r0_c1 = [theta(1);theta(2);theta(3)];
    r1_c2 = [theta(4);theta(5);theta(6)];
    r2_c3 = [theta(7);theta(8);theta(9)];
    r3_c4 = [theta(10);theta(11);theta(12)];
    r4_c5 = [theta(13);theta(14);theta(15)];
    r5_c6 = [theta(16);theta(17);theta(18)];
    r0_1 = [theta(19);theta(20);theta(21)];
    r1_2 = [theta(22);theta(23);theta(24)];
    r2_3 = [theta(25);theta(26);theta(27)];
    r3_4 = [theta(28);theta(29);theta(30)];
    r4_5 = [theta(31);theta(32);theta(33)];
    r5_6 = [theta(34);theta(35);theta(36)];
    r1_c1 = r0_1 - r0_c1;
    r2_c2 = r1_2 - r1_c2;
    r3_c3 = r2_3 - r2_c3;
    r4_c4 = r3_4 - r3_c4;
    r5_c5 = r4_5 - r4_c5;
    r6_c6 = r5_6 - r5_c6;
    
    %z0 axis orientation
    z0 = [0;0;1];
    
    %Gravity
    g0 = [0;0;-9.81];
    
    %Inertia parameters
    I1 =[theta(37),theta(38),theta(39);theta(40),theta(41),theta(42);theta(43),theta(44),theta(45)];
    I2 =[theta(46),theta(47),theta(48);theta(49),theta(50),theta(51);theta(52),theta(53),theta(54)];
    I3 =[theta(55),theta(56),theta(57);theta(58),theta(59),theta(60);theta(61),theta(62),theta(63)];
    I4 =[theta(64),theta(65),theta(66);theta(67),theta(68),theta(69);theta(70),theta(71),theta(72)];
    I5 =[theta(73),theta(74),theta(75);theta(76),theta(77),theta(78);theta(79),theta(80),theta(81)];
    I6 =[theta(82),theta(83),theta(84);theta(85),theta(86),theta(87);theta(88),theta(89),theta(90)];
    
    %Mass parameters
    m1 = theta(91);
    m2 = theta(92);
    m3 = theta(93);
    m4 = theta(94);
    m5 = theta(95);
    m6 = theta(96);
    
    
    %Rotation matrix
    R0_1 = ([cos(q(1)-pi/2) -sin(q(1)-pi/2) 0;sin(q(1)-pi/2) cos(q(1)-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R1_2 = ([cos(q(2)+pi/2) -sin(q(2)+pi/2) 0;sin(q(2)+pi/2) cos(q(2)+pi/2) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R2_3 = ([cos(q(3)) -sin(q(3)) 0;sin(q(3)) cos(q(3)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R3_4 = ([cos(q(4)+pi/2) -sin(q(4)+pi/2) 0;sin(q(4)+pi/2) cos(q(4)+pi/2) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R4_5 = ([cos(q(5)+pi) -sin(q(5)+pi) 0;sin(q(5)+pi) cos(q(5)+pi) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R5_6 = ([cos(q(6)) -sin(q(6)) 0;sin(q(6)) cos(q(6)) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R6_7 = eye(3);
    R0_2 = R0_1*R1_2;
    R0_3 = R0_2*R2_3;
    R0_4 = R0_3*R3_4;
    R0_5 = R0_4*R4_5;
    R0_6 = R0_5*R5_6;
    
    %B parameters
    b1 = R0_1'*z0;     
    b2 = R0_2'*R0_1*z0;
    b3 = R0_3'*R0_2*z0;
    b4 = R0_4'*R0_3*z0;
    b5 = R0_5'*R0_4*z0;
    b6 = R0_6'*R0_5*z0;
    
    %Forward Recursion
    
    %Link1
    omega1 = omega(1);
    alpha1 = b1*d_d_q(1) + cross(omega1,b1*d_q(1));
    a_c1 = cross(d_omega(1),r0_1) + cross(omega1,cross(omega1,r0_1));
    a_e1 = cross(d_omega(1),r0_c1) + cross(omega1,cross(omega1,r0_c1));
    
    %Link2
    omega2 = omega(2);
    alpha2 = R1_2'*alpha1 +b2*d_d_q(2) + cross(omega2,b2*d_q(2));
    a_c2 = R1_2'*a_c1 + cross(d_omega(2),r1_2) + cross(omega2,cross(omega2,r1_2));
    a_e2 = R1_2'*a_e1 + cross(d_omega(2),r1_c2) + cross(omega2,cross(omega2,r1_c2));

    %Link3
    omega3 = omega(3);
    alpha3 = R2_3'*alpha2 +b3*d_d_q(3) + cross(omega3,b3*d_q(3));
    a_c3 = R2_3'*a_c2 + cross(d_omega(2),r2_3) + cross(omega3,cross(omega3,r2_3));
    a_e3 = R2_3'*a_e2 + cross(d_omega(2),r2_c3) + cross(omega3,cross(omega3,r2_c3));    
    
    %Link4
    omega4 = omega(4);
    alpha4 = R3_4'*alpha3 +b4*d_d_q(4) + cross(omega4,b4*d_q(4));
    a_c4 = R3_4'*a_c3 + cross(d_omega(3),r3_4) + cross(omega4,cross(omega4,r3_4));
    a_e4 = R3_4'*a_e3 + cross(d_omega(3),r3_c4) + cross(omega4,cross(omega4,r3_c4));

    %Link5
    omega5 = omega(5);
    alpha5 = R4_5'*alpha4 +b5*d_d_q(5) + cross(omega5,b5*d_q(5));
    a_c5 = R4_5'*a_c4 + cross(d_omega(4),r4_5) + cross(omega5,cross(omega5,r4_5));
    a_e5 = R4_5'*a_e4 + cross(d_omega(4),r4_c5) + cross(omega5,cross(omega5,r4_c5));
    
    %Link6
    omega6 = omega(6);
    alpha6 = R5_6'*alpha5 +b5*d_d_q(6) + cross(omega6,b6*d_q(6));
    a_c6 = R5_6'*a_c5 + cross(d_omega(5),r5_6) + cross(omega6,cross(omega6,r5_6));
    %a_e6 = R5_6'*a_e5 + cross(d_omega(5),r5_c6) + cross(omega6,cross(omega6,r5_c6));

    %Recursão para trás
    
    %Link6
    g6 = R0_6'*g0;
    f6 = R6_7'*f7 + m6*a_c6 - m6*g6;
    t6 = R6_7'*t7 - cross(f6,r5_c6) + cross(R6_7*f7,r6_c6) + cross(omega6,I6*omega6) + I6*alpha6;
    t6dyn = b6'*t6;
    
    %Link5
    g5 = R0_5'*g0;
    f5 = R5_6'*f6 + m5*a_c5 - m5*g5;
    t5 = R5_6'*t6 - cross(f5,r4_c5) + cross(R5_6*f6,r5_c5) + cross(omega5,I5*omega5) + I5*alpha5;
    t5dyn = b5'*t5;
    
    %Link4
    g4 = R0_4'*g0;
    f4 = R4_5'*f5 + m4*a_c4 - m4*g4;
    t4 = R4_5'*t5 - cross(f4,r3_c4) + cross(R4_5*f5,r4_c4) + cross(omega4,I4*omega4) + I4*alpha4;
    t4dyn = b4'*t4;
    
    %Link3
    g3 = R0_3'*g0;
    f3 = R3_4'*f4 + m3*a_c3 - m3*g3;
    t3 = R3_4'*t4 - cross(f3,r2_c3) + cross(R3_4*f4,r3_c3) + cross(omega3,I3*omega3) + I3*alpha3;
    t3dyn = b3'*t3;
    
    %Link2
    g2 = R0_2'*g0;
    f2 = R2_3'*f3 + m2*a_c2 - m2*g2;
    t2 = R2_3'*t3 - cross(f2,r1_c2) + cross(R2_3*f3,r2_c2) + cross(omega2,I2*omega2) + I2*alpha2;
    t2dyn = b2'*t2;
    
    %Link1
    g1 = R0_1'*g0;
    f1 = R1_2'*f2 + m1*a_c1 - m1*g1;
    t1 = R1_2'*t2 - cross(f1,r0_c1) + cross(R1_2*f2,r1_c1) + cross(omega1,I1*omega1) + I1*alpha1;
    t1dyn = b1'*t1;
    
    %Output
    Tau = [t1dyn;t2dyn;t3dyn;t4dyn;t5dyn;t6dyn];
end