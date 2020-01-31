function [Tau] = Symbolic_Dynamic_2DF()
    % Given the dynamic parameters, force and torque in TCP and the output
    % data from Data_Preparation.m file, this function give the torque in
    % each of the 6 joint motors using the Newton-Euler Formulation.

    %theta = [r0_c1;r1_c2;r2_c3;r3_c4;r4_c5;r6_c5;r0_1;r1_2;r2_3;r3_4;r4_5;
    %         r5_6;I1;I2;I3;I4;I5;I6;m1;m2;m3;m4;m5;m6]
    %All the 'r'are vectors (3x1), I1 to I6 are Inertia Matrix (6x6).
    %f7 and t7 are end-effector relative and both are vectors (3X1).
    %Inertia matrix are all symetric. 

    %z0 axis orientation
    z0 = [0;0;1];

    %Gravity
    g0 = [0;-sym('g','real');0];

    %ri_cj is the distance between frame i and body j in meters.
    %This is the body center of mass. Because body j is located at i frame.
    
    lc1 = [sym('lc1','real');0;0];
    lc2 = [sym('lc2','real');0;0];

    %ri_i-1 is the distance between frame i and frame i-1.
    l1 = [sym('l1','real');0;0];
    %r1_2 = [sym('r1_2','real');0;0];
    
    %Inertia parameters
    I1 = sym('I1',[3 3],'real');
    I2 = sym('I2',[3 3],'real');

    %Mass parameters
    m1 = sym('m1','real');
    m2 = sym('m2','real');
    
    %Joint friction coefficients
    b1 = [0,0,sym('b1','real')];
    b2 = [0,0,sym('b2','real')];

    q = sym('q',[1 2],'real');
    dq = sym('dq',[2 1],'real');
    dq = [0;0;dq(1,1);0;0;dq(2,1)];
    ddq = sym('ddq',[2 1],'real');
    ddq = [0;0;ddq(1,1);0;0;ddq(2,1)];
%     
%     g0 = [0;-10;0];
%     lc1 = [1.5;0;0];
%     lc2 = [2;0;0];
%     l1 = [2;0;0];
%     I1 = [0.2,0,0;0,0.2,0;0,0,0.2];
%     I2 = [0.1,0,0;0,0.1,0;0,0,0.1];
%     m1 = 0.5;
%     m2 = 0.3;
%     b1 = [0,0,0.05];
%     b2 = [0,0,0.05];
%     q1 = pi/4;
%     q2 = -pi/8;
%     q = [q1,q2];
%     dq1 = 1;
%     dq2 = 2;
%     dq = [0;0;dq1;0;0;dq2];
%     ddq1 = 0.5;
%     ddq2 = 0.2;
%     ddq = [0;0;ddq1;0;0;ddq2];
    
        %Rotation matrix. Here, the argument in the cos and sin are
        %relative to the configuration where all joint angles are zero
        R0_1 = ([cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
        R1_2 = ([cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
        %RJ0_1 = [cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1];
        %RJ1_2 = [cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1];
        R0_2 = R0_1*R1_2;
        
        %ri+1_cj is the distance between the body j and the previous frame.
        %Here, is necessary to rotate the distance between the frames by
        %the joint angle to maintain coherence.
        rc1_1 = lc1 - l1;
        %r1_c2 = lc2 - RJ1_2*r1_2;
        
        %Forward Recursion

        %Body0
        a_e1 = cross(ddq(1:3,1),l1) + cross(dq(1:3,1),cross(dq(1:3,1),l1));
        a_c1 = cross(ddq(1:3,1),lc1) + cross(dq(1:3,1),cross(dq(1:3,1),lc1));

        %Body1
        a_c2 = R1_2'*a_e1 + cross(ddq(4:6,1),lc2) + cross(dq(4:6,1),cross(dq(4:6,1),lc2));

        %Backward Recursion

        %Body1
        g2 = R0_2'*g0;
        f2 = m2*a_c2 - m2*g2;
        t2 = - cross(f2,lc2) + cross(dq(4:6,1),I2*dq(4:6,1)) + I2*ddq(4:6,1) + [0;0;b2*dq(4:6,1)];
        t2dyn = z0'*t2;
        %t2sym = sym('t2s',[3 1],'real');
        %Body0
        g1 = R0_1'*g0;
        f1 = R1_2*f2 + m1*a_c1 - m1*g1;
        t1 = R1_2*t2 - cross(f1,lc1) + cross(R1_2*f2,rc1_1) + cross(dq(1:3,1),I1*dq(1:3,1)) + I1*ddq(1:3,1) + [0;0;b1*dq(1:3,1)];
        t1dyn = z0'*t1;

    %Output
    Tau = [t1dyn;t2dyn];
end