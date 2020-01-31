function [Tau] = Symbolic_Dynamic()
    % Given the dynamic parameters, force and torque in TCP and the output
    % data from Data_Preparation.m file, this function give the torque in
    % each of the 6 joint motors using the Newton-Euler Formulation.

    %theta = [r0_c1;r1_c2;r2_c3;r3_c4;r4_c5;r6_c5;r0_1;r1_2;r2_3;r3_4;r4_5;
    %         r5_6;I1;I2;I3;I4;I5;I6;m1;m2;m3;m4;m5;m6]
    %All the 'r'are vectors (3x1), I1 to I6 are Inertia Matrix (6x6).
    %f7 and t7 are end-effector relative and both are vectors (3X1).
    %Inertia matrix are all symetric. 

    addpath '~/Documents/Git/UR3-Modelation/Data Manipulations';
    
    %z0 axis orientation
    z0 = [0;0;1];

    %Gravity
    g0 = [0;0;-9.81];

    %ri_cj is the distance between frame i and body j in meters.
    %This is the body center of mass. Because body j is located at i frame.
    r0_c0 = sym('r0_c0',[3 1]);
    r1_c1 = sym('r1_c1',[3 1]);
    r2_c2 = sym('r2_c2',[3 1]);
    r3_c3 = sym('r3_c3',[3 1]);
    r4_c4 = sym('r4_c4',[3 1]);
    r5_c5 = sym('r5_c5',[3 1]);

    %ri_i-1 is the distance between frame i and frame i-1.
    r0_1 = [0;0;0.152];
    r1_2 = [0.244;0;0];
    r2_3 = [0.213;0;0];
    r3_4 = [0;0;0.112];
    r4_5 = [0;0;0.085];
    r5_6 = [0;0;0.082];

    %Inertia parameters
    I0 = sym('I0',[3 3]);
    I1 = sym('I1',[3 3]);
    I2 = sym('I2',[3 3]);
    I3 = sym('I3',[3 3]);
    I4 = sym('I4',[3 3]);
    I5 = sym('I5',[3 3]);

    %Mass parameters
    m0 = sym('m0');
    m1 = sym('m1');
    m2 = sym('m2');
    m3 = sym('m3');
    m4 = sym('m4');
    m5 = sym('m5');
    
    %Joint friction coefficients
    b0 = [0,0,sym('b0')];
    b1 = [0,0,sym('b1')];
    b2 = [0,0,sym('b2')];
    b3 = [0,0,sym('b3')];
    b4 = [0,0,sym('b4')];
    b5 = [0,0,sym('b5')];

    q = sym('q',[1 6]);
    q(1,1) = 0;
    q(1,2) = 0;
    q(1,3) = 0;
    q(1,4) = 0;
    omega = sym('omega',[18 1]);
    omega(1,1) = 0;
    omega(1,2) = 0;
    omega(1,3) = 0;
    omega(1,4) = 0;
    omega(1,5) = 0;
    omega(1,6) = 0;
    omega(1,7) = 0;
    omega(1,8) = 0;
    omega(1,9) = 0;
    omega(1,10) = 0;
    omega(1,11) = 0;
    omega(1,12) = 0;
    
    d_omega = sym('d_omega',[18 1]);
    d_omega(1,1) = 0;
    d_omega(1,2) = 0;
    d_omega(1,3) = 0;
    d_omega(1,4) = 0;
    d_omega(1,5) = 0;
    d_omega(1,6) = 0;
    d_omega(1,7) = 0;
    d_omega(1,8) = 0;
    d_omega(1,9) = 0;
    d_omega(1,10) = 0;
    d_omega(1,11) = 0;
    d_omega(1,12) = 0;
    
    alpha = sym('alpha',[18 1]);
    alpha(1,1) = 0;
    alpha(1,2) = 0;
    alpha(1,3) = 0;
    alpha(1,4) = 0;
    alpha(1,5) = 0;
    alpha(1,6) = 0;
    alpha(1,7) = 0;
    alpha(1,8) = 0;
    alpha(1,9) = 0;
    alpha(1,10) = 0;
    alpha(1,11) = 0;
    alpha(1,12) = 0;
        %Rotation matrix. Here, the argument in the cos and sin are
        %relative to the configuration where all joint angles are zero
        [R0_1,R1_2,R2_3,R3_4,R4_5,R5_6] = Rotation_Matrices(q);

        R0_2 = R0_1*R1_2;
        R0_3 = R0_2*R2_3;
        R0_4 = R0_3*R3_4;
        R0_5 = R0_4*R4_5;
        RJ0_1 = [cos(q(1,1)) -sin(q(1,1)) 0;sin(q(1,1)) cos(q(1,1)) 0; 0 0 1];
        RJ1_2 = [cos(q(1,2)) -sin(q(1,2)) 0;sin(q(1,2)) cos(q(1,2)) 0; 0 0 1];
        RJ2_3 = [cos(q(1,3)) -sin(q(1,3)) 0;sin(q(1,3)) cos(q(1,3)) 0; 0 0 1];
        RJ3_4 = [cos(q(1,4)) -sin(q(1,4)) 0;sin(q(1,4)) cos(q(1,4)) 0; 0 0 1];
        RJ4_5 = [cos(q(1,5)) -sin(q(1,5)) 0;sin(q(1,5)) cos(q(1,5)) 0; 0 0 1];
        RJ5_6 = [cos(q(1,6)) -sin(q(1,6)) 0;sin(q(1,6)) cos(q(1,6)) 0; 0 0 1];

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
        a_e0 = cross(d_omega(1:3,1),r0_1) + cross(omega(1:3,1),cross(omega(1:3,1),r0_1));
        a_c0 = cross(d_omega(1:3,1),r0_c0) + cross(omega(1:3,1),cross(omega(1:3,1),r0_c0));

        %Body1
        a_e1 = R0_1'*a_e0 + cross(d_omega(4:6,1),r1_2) + cross(omega(4:6,1),cross(omega(4:6,1),r1_2));
        a_c1 = R0_1'*a_e0 + cross(d_omega(4:6,1),r1_c1) + cross(omega(4:6,1),cross(omega(4:6,1),r1_c1));

        %Body2
        a_e2 = R1_2'*a_e1 + cross(d_omega(7:9,1),r2_3) + cross(omega(7:9,1),cross(omega(7:9,1),r2_3));
        a_c2 = R1_2'*a_e1 + cross(d_omega(7:9,1),r2_c2) + cross(omega(7:9,1),cross(omega(7:9,1),r2_c2));

        %Body3
        a_e3 = R2_3'*a_e2 + cross(d_omega(10:12,1),r3_4) + cross(omega(10:12,1),cross(omega(10:12,1),r3_4));
        a_c3 = R2_3'*a_e2 + cross(d_omega(10:12,1),r3_c3) + cross(omega(10:12,1),cross(omega(10:12,1),r3_c3));

        %Body4
        a_e4 = R3_4'*a_e3 + cross(d_omega(13:15,1),r4_5) + cross(omega(13:15,1),cross(omega(13:15,1),r4_5));
        a_c4 = R3_4'*a_e3 + cross(d_omega(13:15,1),r4_c4) + cross(omega(13:15,1),cross(omega(13:15,1),r4_c4));

        %Body5
        a_c5 = R4_5'*a_e4 + cross(d_omega(16:18,1),r5_c5) + cross(omega(16:18,1),cross(omega(16:18,1),r5_c5));

        %Backward Recursion

        %Body5
        g5 = R0_5'*g0;
        f5 = m5*a_c5 - m5*g5;
        t5 = - cross(f5,r5_c5) + cross(omega(16:18,1),I5*omega(16:18,1)) + I5*alpha(16:18,1) + [0;0;b5*omega(16:18,1)];
        t5dyn = z0'*t5;

        %Body4
        g4 = R0_4'*g0;
        f4 = R4_5*f5 + m4*a_c4 - m4*g4;
        t4 = R4_5*t5 - cross(f4,r4_c4) + cross(R4_5*f5,r5_c4) + cross(omega(13:15,1),I4*omega(13:15,1)) + I4*alpha(13:15,1) + [0;0;b4*omega(13:15,1)];
        t4dyn = z0'*t4;

        %Body3
        g3 = R0_3'*g0;
        f3 = R3_4*f4 + m3*a_c3 - m3*g3;
        t3 = R3_4*t4 - cross(f3,r3_c3) + cross(R3_4*f4,r4_c3) + cross(omega(10:12,1),I3*omega(10:12,1)) + I3*alpha(10:12,1) + [0;0;b3*omega(10:12,1)];
        t3dyn = z0'*t3;

        %Body2
        g2 = R0_2'*g0;
        f2 = R2_3*f3 + m2*a_c2 - m2*g2;
        t2 = R2_3*t3 - cross(f2,r2_c2) + cross(R2_3*f3,r3_c2) + cross(omega(7:9,1),I2*omega(7:9,1)) + I2*alpha(7:9,1) + [0;0;b2*omega(7:9,1)];
        t2dyn = z0'*t2;

        %Body1
        g1 = R0_1'*g0;
        f1 = R1_2*f2 + m1*a_c1 - m1*g1;
        t1 = R1_2*t2 - cross(f1,r1_c1) + cross(R1_2*f2,r2_c1) + cross(omega(4:6,1),I1*omega(4:6,1)) + I1*alpha(4:6,1) + [0;0;b1*omega(4:6,1)];
        t1dyn = z0'*t1;

        %Body0
        f0 = R0_1*f1 + m0*a_c0 - m0*g0;
        t0 = R0_1*t1 - cross(f0,r0_c0) + cross(R0_1*f1,r1_c0) + cross(omega(1:3,1),I0*omega(1:3,1)) + I0*alpha(1:3,1) + [0;0;b0*omega(1:3,1)];
        t0dyn = z0'*t0;

    %Output
    Tau = [t0dyn;t1dyn;t2dyn;t3dyn;t4dyn;t5dyn]';
end