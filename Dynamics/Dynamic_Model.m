function [Tau] = Dynamic_Model(theta,q,omega,d_omega,alpha,~,~)
    %Not in use!

    % Given the dynamic parameters, force and torque in TCP and the output
    % data from Data_Preparation.m file, this function give the torque in
    % each of the 6 joint motors using the Newton-Euler Formulation.

    %theta = [r0_c1;r1_c2;r2_c3;r3_c4;r4_c5;r6_c5;r0_1;r1_2;r2_3;r3_4;r4_5;
    %         r5_6;I1;I2;I3;I4;I5;I6;m1;m2;m3;m4;m5;m6]
    %All the 'r'are vectors (3x1), I1 to I6 are Inertia Matrix (6x6).
    %f7 and t7 are end-effector relative and both are vectors (3X1).
    %Inertia matrix are all symetric. 

    f6 = [0;0;0];
    t6 = [0;0;0];
    
    addpath '~/Documents/Git/UR3-Modelation/Data Manipulations';

    %Pre Alocation
    
    t0dyn = zeros(1,length(q));
    t1dyn = zeros(1,length(q));
    t2dyn = zeros(1,length(q));
    t3dyn = zeros(1,length(q));
    t4dyn = zeros(1,length(q));
    t5dyn = zeros(1,length(q));

    %z0 axis orientation
    z0 = [0;0;1];

    %Gravity
    g0 = [0;0;-9.81];

    %ri_cj is the distance between frame i and body j in meters.
    %This is the body center of mass. Because body j is located at i frame.
    r0_c0 = [theta(1);theta(2);theta(3)];
    r1_c1 = [theta(4);theta(5);theta(6)];
    r2_c2 = [theta(7);theta(8);theta(9)];
    r3_c3 = [theta(10);theta(11);theta(12)];
    r4_c4 = [theta(13);theta(14);theta(15)];
    r5_c5 = [theta(16);theta(17);theta(18)];

    %ri_i-1 is the distance between frame i and frame i-1.
    r0_1 = [0;0;0.152];
    r1_2 = [0.244;0;0];
    r2_3 = [0.213;0;0];
    r3_4 = [0;0;0.112];
    r4_5 = [0;0;0.085];
    r5_6 = [0;0;0.082];

    %Inertia parameters
    I0 =[theta(19),theta(20),theta(21);theta(20),theta(22),theta(23);theta(21),theta(23),theta(24)];
    I1 =[theta(25),theta(26),theta(27);theta(26),theta(28),theta(29);theta(27),theta(29),theta(30)];
    I2 =[theta(31),theta(32),theta(33);theta(32),theta(34),theta(35);theta(33),theta(35),theta(36)];
    I3 =[theta(37),theta(38),theta(39);theta(38),theta(40),theta(41);theta(39),theta(41),theta(42)];
    I4 =[theta(43),theta(44),theta(45);theta(44),theta(46),theta(47);theta(45),theta(47),theta(48)];
    I5 =[theta(49),theta(50),theta(51);theta(50),theta(52),theta(53);theta(51),theta(53),theta(54)];

    %Mass parameters
    m0 = theta(55);
    m1 = theta(56);
    m2 = theta(57);
    m3 = theta(58);
    m4 = theta(59);
    m5 = theta(60);
    
    %Joint friction coefficients
    b0 = [0,0,theta(61)];
    b1 = [0,0,theta(62)];
    b2 = [0,0,theta(63)];
    b3 = [0,0,theta(64)];
    b4 = [0,0,theta(65)];
    b5 = [0,0,theta(66)];

    for i=1:length(q)-1

        %Rotation matrix. Here, the argument in the cos and sin are
        %relative to the configuration where all joint angles are zero
        [R0_1,R1_2,R2_3,R3_4,R4_5,R5_6] = Rotation_Matrices(q(i,:));

        R0_2 = R0_1*R1_2;
        R0_3 = R0_2*R2_3;
        R0_4 = R0_3*R3_4;
        R0_5 = R0_4*R4_5;
        RJ0_1 = [cos(q(i,1)) -sin(q(i,1)) 0;sin(q(i,1)) cos(q(i,1)) 0; 0 0 1];
        RJ1_2 = [cos(q(i,2)) -sin(q(i,2)) 0;sin(q(i,2)) cos(q(i,2)) 0; 0 0 1];
        RJ2_3 = [cos(q(i,3)) -sin(q(i,3)) 0;sin(q(i,3)) cos(q(i,3)) 0; 0 0 1];
        RJ3_4 = [cos(q(i,4)) -sin(q(i,4)) 0;sin(q(i,4)) cos(q(i,4)) 0; 0 0 1];
        RJ4_5 = [cos(q(i,5)) -sin(q(i,5)) 0;sin(q(i,5)) cos(q(i,5)) 0; 0 0 1];
        RJ5_6 = [cos(q(i,6)) -sin(q(i,6)) 0;sin(q(i,6)) cos(q(i,6)) 0; 0 0 1];

        %ri+1_cj is the distance between the body j and the previous frame.
        %Here, is necessary to rotate the distance between the frames by
        %the joint angle to maintain coherence.
        r1_c0 = r0_c0 - RJ0_1*r0_1;
        r2_c1 = r1_c1 - RJ1_2*r1_2;
        r3_c2 = r2_c2 - RJ2_3*r2_3;
        r4_c3 = r3_c3 - RJ3_4*r3_4;
        r5_c4 = r4_c4 - RJ4_5*r4_5;
        r6_c5 = r5_c5 - RJ5_6*r5_6;      

        %Forward Recursion

        %Body0
        a_e0 = cross(d_omega(1:3,i),r0_1) + cross(omega(1:3,i),cross(omega(1:3,i),r0_1));
        a_c0 = cross(d_omega(1:3,i),r0_c0) + cross(omega(1:3,i),cross(omega(1:3,i),r0_c0));

        %Body1
        a_e1 = R0_1'*a_e0 + cross(d_omega(4:6,i),r1_2) + cross(omega(4:6,i),cross(omega(4:6,i),r1_2));
        a_c1 = R0_1'*a_e0 + cross(d_omega(4:6,i),r1_c1) + cross(omega(4:6,i),cross(omega(4:6,i),r1_c1));

        %Body2
        a_e2 = R1_2'*a_e1 + cross(d_omega(7:9,i),r2_3) + cross(omega(7:9,i),cross(omega(7:9,i),r2_3));
        a_c2 = R1_2'*a_e1 + cross(d_omega(7:9,i),r2_c2) + cross(omega(7:9,i),cross(omega(7:9,i),r2_c2));

        %Body3
        a_e3 = R2_3'*a_e2 + cross(d_omega(10:12,i),r3_4) + cross(omega(10:12,i),cross(omega(10:12,i),r3_4));
        a_c3 = R2_3'*a_e2 + cross(d_omega(10:12,i),r3_c3) + cross(omega(10:12,i),cross(omega(10:12,i),r3_c3));

        %Body4
        a_e4 = R3_4'*a_e3 + cross(d_omega(13:15,i),r4_5) + cross(omega(13:15,i),cross(omega(13:15,i),r4_5));
        a_c4 = R3_4'*a_e3 + cross(d_omega(13:15,i),r4_c4) + cross(omega(13:15,i),cross(omega(13:15,i),r4_c4));

        %Body5
        a_c5 = R4_5'*a_e4 + cross(d_omega(16:18,i),r5_c5) + cross(omega(16:18,i),cross(omega(16:18,i),r5_c5));

        %Backward Recursion

        %Body5
        g5 = R0_5'*g0;
        f5 = R5_6*f6 + m5*a_c5 - m5*g5;
        t5 = R5_6*t6 - cross(f5,r5_c5) + cross(R5_6*f6,r6_c5) + cross(omega(16:18,i),I5*omega(16:18,i)) + I5*alpha(16:18,i) + [0;0;b5*omega(16:18,i)];
        t5dyn(i) = z0'*t5;

        %Body4
        g4 = R0_4'*g0;
        f4 = R4_5*f5 + m4*a_c4 - m4*g4;
        t4 = R4_5*t5 - cross(f4,r4_c4) + cross(R4_5*f5,r5_c4) + cross(omega(13:15,i),I4*omega(13:15,i)) + I4*alpha(13:15,i) + [0;0;b4*omega(13:15,i)];
        t4dyn(i) = z0'*t4;

        %Body3
        g3 = R0_3'*g0;
        f3 = R3_4*f4 + m3*a_c3 - m3*g3;
        t3 = R3_4*t4 - cross(f3,r3_c3) + cross(R3_4*f4,r4_c3) + cross(omega(10:12,i),I3*omega(10:12,i)) + I3*alpha(10:12,i) + [0;0;b3*omega(10:12,i)];
        t3dyn(i) = z0'*t3;

        %Body2
        g2 = R0_2'*g0;
        f2 = R2_3*f3 + m2*a_c2 - m2*g2;
        t2 = R2_3*t3 - cross(f2,r2_c2) + cross(R2_3*f3,r3_c2) + cross(omega(7:9,i),I2*omega(7:9,i)) + I2*alpha(7:9,i) + [0;0;b2*omega(7:9,i)];
        t2dyn(i) = z0'*t2;

        %Body1
        g1 = R0_1'*g0;
        f1 = R1_2*f2 + m1*a_c1 - m1*g1;
        t1 = R1_2*t2 - cross(f1,r1_c1) + cross(R1_2*f2,r2_c1) + cross(omega(4:6,i),I1*omega(4:6,i)) + I1*alpha(4:6,i) + [0;0;b1*omega(4:6,i)];
        t1dyn(i) = z0'*t1;

        %Body0
        f0 = R0_1*f1 + m0*a_c0 - m0*g0;
        t0 = R0_1*t1 - cross(f0,r0_c0) + cross(R0_1*f1,r1_c0) + cross(omega(1:3,i),I0*omega(1:3,i)) + I0*alpha(1:3,i) + [0;0;b0*omega(1:3,i)];
        t0dyn(i) = z0'*t0;
    end
    %Output
    Tau = [t0dyn;t1dyn;t2dyn;t3dyn;t4dyn;t5dyn]';
end