function [Tau] = DOF5()
    q = sym('q',[5 1],'real');
    dq = sym('dq',[5 1],'real');
    ddq = sym('ddq',[5 1],'real');

    z0 = [0;0;1];

    R0_1 = [cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R1_2 = [cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1];
    R2_3 = [cos(q(3)) -sin(q(3)) 0;sin(q(3)) cos(q(3)) 0; 0 0 1];
    R3_4 = [cos(q(4)) -sin(q(4)) 0;sin(q(4)) cos(q(4)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R4_5 = [cos(q(5)) -sin(q(5)) 0;sin(q(5)) cos(q(5)) 0; 0 0 1]*round([1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R0_2 = simplify(R0_1*R1_2);
    R0_3 = simplify(R0_2*R2_3);
    R0_4 = simplify(R0_3*R3_4);
    R0_5 = simplify(R0_4*R4_5);

    % variáveis estáticas para simplificar as expressões abaixo
    b1 = simplify(R0_1'*z0);
    b2 = simplify(R0_2'*R0_1*z0);
    b3 = simplify(R0_3'*R0_2*z0);
    b4 = simplify(R0_4'*R0_3*z0);
    b5 = simplify(R0_5'*R0_4*z0);
    
    omega1 = b1*dq(1);
    omega2 = R1_2'*omega1 + b2*dq(2);
    omega3 = R2_3'*omega2 + b3*dq(3);
    omega4 = R3_4'*omega3 + b4*dq(4);
    omega5 = R4_5'*omega4 + b5*dq(5);
    
    alpha1 = b1*ddq(1) + cross(omega1,b1*dq(1));
    alpha2 = R1_2'*alpha1 + b2*ddq(2) + cross(omega2,b2*dq(2));
    alpha3 = R2_3'*alpha2 + b3*ddq(3) + cross(omega3,b3*dq(3));
    alpha4 = R3_4'*alpha3 + b4*ddq(4) + cross(omega4,b4*dq(4));
    alpha5 = R4_5'*alpha4 + b5*ddq(5) + cross(omega5,b5*dq(5));
    
    g0 = [0;0;-sym('g','real')];
    lc1 = [0;sym('lc1y','real');0]; %Distancia entre o frame 0 e o centro de massa 1 em relação ao frame 1
    lc2 = [-sym('lc2x','real');0;sym('lc2z','real')];
    lc3 = [-sym('lc3x','real');0;-sym('lc3z','real')];
    lc4 = [0;sym('lc4y','real');0];
    lc5 = [0;-sym('lc5y','real');0];
    l1 = [0;sym('l1y','real');0];
    l2 = [-sym('l2x','real');0;sym('l2z','real')];
    l3 = [-sym('l3x','real');0;-sym('l3z','real')];
    l4 = [0;sym('l4y','real');0];
    rc1_1 = lc1 - l1;
    rc2_2 = lc2 - l2;
    rc3_3 = lc3 - l3;
    rc4_4 = lc4 - l4;
    I1 = [sym('I1x','real'),0,0;0,sym('I1y','real'),0;0,0,sym('I1z','real')];% Considerando apenas a diag principal
    I2 = [sym('I2x','real'),0,0;0,sym('I2y','real'),0;0,0,sym('I2z','real')];% Considerando apenas a diag principal
    I3 = [sym('I3x','real'),0,0;0,sym('I3y','real'),0;0,0,sym('I3z','real')];% Considerando apenas a diag principal
    I4 = [sym('I4x','real'),0,0;0,sym('I4y','real'),0;0,0,sym('I4z','real')];% Considerando apenas a diag principal
    I5 = [sym('I5x','real'),0,0;0,sym('I5y','real'),0;0,0,sym('I5z','real')];% Considerando apenas a diag principal
    m1 = sym('m1','real');
    m2 = sym('m2','real');
    m3 = sym('m3','real');
    m4 = sym('m4','real');
    m5 = sym('m5','real');
    beta1 = sym('beta1','real');
    beta2 = sym('beta2','real');
    beta3 = sym('beta3','real');
    beta4 = sym('beta4','real');
    beta5 = sym('beta5','real');
    
    a_c1 = cross(alpha1,lc1) + cross(omega1,cross(omega1,lc1));
    a_e1 = cross(alpha1,l1) + cross(omega1,cross(omega1,l1));
    
    a_c2 = R1_2'*a_e1 + cross(alpha2,lc2) + cross(omega2,cross(omega2,lc2));
    a_e2 = R1_2'*a_e1 + cross(alpha2,l2) + cross(omega2,cross(omega2,l2));
     
    a_c3 = R2_3'*a_e2 + cross(alpha3,lc3) + cross(omega3,cross(omega3,lc3));
    a_e3 = R2_3'*a_e2 + cross(alpha3,l3) + cross(omega3,cross(omega3,l3));
    
    a_c4 = R3_4'*a_e3 + cross(alpha4,lc4) + cross(omega4,cross(omega4,lc4));
    a_e4 = R3_4'*a_e3 + cross(alpha4,l4) + cross(omega4,cross(omega4,l4));
    
    a_c5 = R4_5'*a_e4 + cross(alpha5,lc5) + cross(omega5,cross(omega5,lc5));
    
    g5 = R0_5'*g0;
    f5 = m5*a_c5 - m5*g5;
    t5 = - cross(f5,lc5) + cross(omega5,I5*omega5) + I5*alpha5 + beta5*omega5;
    t5dyn = simplify(b5'*t5);
    
    g4 = R0_4'*g0;
    f4 = R4_5*f5 + m4*a_c4 - m4*g4;
    t4 = R4_5*t5 - cross(f4,lc4) + cross(R4_5*f5,rc4_4) + cross(omega4,I4*omega4) + I4*alpha4 + beta4*omega4;
    t4dyn = simplify(b4'*t4);
    
    g3 = R0_3'*g0;
    f3 = R3_4*f4 + m3*a_c3 - m3*g3;
    t3 = R3_4*t4 - cross(f3,lc3) + cross(R3_4*f4,rc3_3) + cross(omega3,I3*omega3) + I3*alpha3 + beta3*omega3;
    t3dyn = simplify(b3'*t3);
    
    g2 = R0_2'*g0;
    f2 = R2_3*f3 + m2*a_c2 - m2*g2;
    t2 = R2_3*t3 - cross(f2,lc2) + cross(R2_3*f3,rc2_2) + cross(omega2,I2*omega2) + I2*alpha2 + beta2*omega2;
    t2dyn = simplify(b2'*t2);
    
    g1 = R0_1'*g0;
    f1 = R1_2*f2 + m1*a_c1 - m1*g1;
    t1 = R1_2*t2 - cross(f1,lc1) + cross(R1_2*f2,rc1_1) + cross(omega1,I1*omega1) + I1*alpha1 + beta1*omega1; % Torque em relação ao eixo 1
    t1dyn = simplify(b1'*t1);
     
    Tau = [t1dyn;t2dyn;t3dyn;t4dyn;t5dyn];
end