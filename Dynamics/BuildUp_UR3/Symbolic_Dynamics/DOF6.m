function [Dyn] = DOF6()
    % Medidas do robô
    q = sym('q',[6 1],'real');
    dq = sym('dq',[6 1],'real');
    ddq = sym('ddq',[6 1],'real');

    % orientação do eixo z do frame inercial
    z0 = [0;0;1];
    
    %matrizes de rotação entre os eixos
    R0_1 = [cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R1_2 = [cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1];
    R2_3 = [cos(q(3)) -sin(q(3)) 0;sin(q(3)) cos(q(3)) 0; 0 0 1];
    R3_4 = [cos(q(4)) -sin(q(4)) 0;sin(q(4)) cos(q(4)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R4_5 = [cos(q(5)) -sin(q(5)) 0;sin(q(5)) cos(q(5)) 0; 0 0 1]*round([1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R5_6 = [cos(q(6)) -sin(q(6)) 0;sin(q(6)) cos(q(6)) 0; 0 0 1];
    R0_2 = simplify(R0_1*R1_2);
    R0_3 = simplify(R0_2*R2_3);
    R0_4 = simplify(R0_3*R3_4);
    R0_5 = simplify(R0_4*R4_5);
    R0_6 = simplify(R0_5*R5_6);

    % variáveis estáticas para simplificar as expressões abaixo
    b1 = simplify(R0_1'*z0);
    b2 = simplify(R0_2'*R0_1*z0);
    b3 = simplify(R0_3'*R0_2*z0);
    b4 = simplify(R0_4'*R0_3*z0);
    b5 = simplify(R0_5'*R0_4*z0);
    b6 = simplify(R0_6'*R0_5*z0);
    b6 = [0;b6(2);b6(3)]; %simplified via mapple and b6(1) == 0, but matlab couldn't solve
    
    % Velocidade angular de cada eixo
    omega1 = b1*dq(1);
    omega2 = R1_2'*omega1 + b2*dq(2);
    omega3 = R2_3'*omega2 + b3*dq(3);
    omega4 = R3_4'*omega3 + b4*dq(4);
    omega5 = R4_5'*omega4 + b5*dq(5);
    omega6 = R5_6'*omega5 + b6*dq(6);
    
    % Aceleração angular de cada eixo
    alpha1 = b1*ddq(1) + cross(omega1,b1*dq(1));
    alpha2 = R1_2'*alpha1 + b2*ddq(2) + cross(omega2,b2*dq(2));
    alpha3 = R2_3'*alpha2 + b3*ddq(3) + cross(omega3,b3*dq(3));
    alpha4 = R3_4'*alpha3 + b4*ddq(4) + cross(omega4,b4*dq(4));
    alpha5 = R4_5'*alpha4 + b5*ddq(5) + cross(omega5,b5*dq(5));
    alpha6 = R5_6'*alpha5 + b6*ddq(6) + cross(omega6,b6*dq(6));
    
    % Parâmetros constantes do robô
    g0 = [0;0;-sym('g','real')];
    lc1 = [0;sym('lc1y','real');0]; %Distancia entre o frame 0 e o centro de massa 1 em relação ao frame 1
    lc2 = [-sym('lc2x','real');0;sym('lc2z','real')];
    lc3 = [-sym('lc3x','real');0;-sym('lc3z','real')];
    lc4 = [0;sym('lc4y','real');0];
    lc5 = [0;-sym('lc5y','real');0];
    lc6 = [0;0;sym('lc6z','real')];
    l1 = [0;sym('l1y','real');0];
    l2 = [-sym('l2x','real');0;sym('l2z','real')];
    l3 = [-sym('l3x','real');0;-sym('l3z','real')];
    l4 = [0;sym('l4y','real');0];
    l5 = [0;-sym('l5y','real');0];
    rc1_1 = lc1 - l1;
    rc2_2 = lc2 - l2;
    rc3_3 = lc3 - l3;
    rc4_4 = lc4 - l4;
    rc5_5 = lc5 - l5;
    I1 = [sym('I1x','real'),0,0;0,sym('I1y','real'),0;0,0,sym('I1z','real')];% Considerando apenas a diag principal
    I2 = [sym('I2x','real'),0,0;0,sym('I2y','real'),0;0,0,sym('I2z','real')];% Considerando apenas a diag principal
    I3 = [sym('I3x','real'),0,0;0,sym('I3y','real'),0;0,0,sym('I3z','real')];% Considerando apenas a diag principal
    I4 = [sym('I4x','real'),0,0;0,sym('I4y','real'),0;0,0,sym('I4z','real')];% Considerando apenas a diag principal
    I5 = [sym('I5x','real'),0,0;0,sym('I5y','real'),0;0,0,sym('I5z','real')];% Considerando apenas a diag principal
    I6 = [sym('I6x','real'),0,0;0,sym('I6y','real'),0;0,0,sym('I6z','real')];% Considerando apenas a diag principal
    m1 = sym('m1','real');
    m2 = sym('m2','real');
    m3 = sym('m3','real');
    m4 = sym('m4','real');
    m5 = sym('m5','real');
    m6 = sym('m6','real');
    beta1 = sym('beta1','real');
    beta2 = sym('beta2','real');
    beta3 = sym('beta3','real');
    beta4 = sym('beta4','real');
    beta5 = sym('beta5','real');
    beta6 = sym('beta6','real');
    
    % Acelerações lineares do centro de massa e do fim de cada link
    a_c1 = cross(alpha1,lc1) + cross(omega1,cross(omega1,lc1));
    a_e1 = cross(alpha1,l1) + cross(omega1,cross(omega1,l1));
    
    a_c2 = R1_2'*a_e1 + cross(alpha2,lc2) + cross(omega2,cross(omega2,lc2));
    a_e2 = R1_2'*a_e1 + cross(alpha2,l2) + cross(omega2,cross(omega2,l2));
    
    a_c3 = R2_3'*a_e2 + cross(alpha3,lc3) + cross(omega3,cross(omega3,lc3));
    a_e3 = R2_3'*a_e2 + cross(alpha3,l3) + cross(omega3,cross(omega3,l3));
    
    a_c4 = R3_4'*a_e3 + cross(alpha4,lc4) + cross(omega4,cross(omega4,lc4));
    a_e4 = R3_4'*a_e3 + cross(alpha4,l4) + cross(omega4,cross(omega4,l4));
    
    a_c5 = R4_5'*a_e4 + cross(alpha5,lc5) + cross(omega5,cross(omega5,lc5));
    a_e5 = R4_5'*a_e4 + cross(alpha5,l5) + cross(omega5,cross(omega5,l5));
    
    a_c6 = R5_6'*a_e5 + cross(alpha6,lc6) + cross(omega6,cross(omega6,lc6));
    
    % Calculo do balanço de força e torque para cada link
    g6 = R0_6'*g0;
    f6 = m6*a_c6 - m6*g6;
    t6 = - cross(f6,lc6) + I6*alpha6 + cross(omega6,I6*omega6);
    t6dyn = simplify(b6'*t6) + beta6*dq(6);
    
    g5 = R0_5'*g0;
    f5 = R5_6*f6 + m5*a_c5 - m5*g5;
    t5 = R5_6*t6 - cross(f5,lc5) + cross(R5_6*f6,rc5_5) + I5*alpha5 + cross(omega5,I5*omega5);
    t5dyn = simplify(b5'*t5) + beta5*dq(5);
    
    g4 = R0_4'*g0;
    f4 = R4_5*f5 + m4*a_c4 - m4*g4;
    t4 = R4_5*t5 - cross(f4,lc4) + cross(R4_5*f5,rc4_4) + I4*alpha4 + cross(omega4,I4*omega4);
    t4dyn = simplify(b4'*t4) + beta4*dq(4);
    
    g3 = R0_3'*g0;
    f3 = R3_4*f4 + m3*a_c3 - m3*g3;
    t3 = R3_4*t4 - cross(f3,lc3) + cross(R3_4*f4,rc3_3) + I3*alpha3 + cross(omega3,I3*omega3);
    t3dyn = simplify(b3'*t3) + beta3*dq(3);
    
    g2 = R0_2'*g0;
    f2 = R2_3*f3 + m2*a_c2 - m2*g2;
    t2 = R2_3*t3 - cross(f2,lc2) + cross(R2_3*f3,rc2_2) + I2*alpha2 + cross(omega2,I2*omega2);
    t2dyn = simplify(b2'*t2) + beta2*dq(2);
    
    g1 = R0_1'*g0;
    f1 = R1_2*f2 + m1*a_c1 - m1*g1;
    t1 = R1_2*t2 - cross(f1,lc1) + cross(R1_2*f2,rc1_1) + I1*alpha1 + cross(omega1,I1*omega1); 
    t1dyn = simplify(b1'*t1) + beta1*dq(1);
     
    Tau = [t1dyn;t2dyn;t3dyn;t4dyn;t5dyn;t6dyn];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Teste para Montar matrizes%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    m1_1 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[1 0 0 0 0 0 0 0 0 0 0 0 0]);
    m1_2 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 1 0 0 0 0 0 0 0 0 0 0 0]);
    m1_3 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 1 0 0 0 0 0 0 0 0 0 0]);
    m1_4 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 1 0 0 0 0 0 0 0 0 0]);
    m1_5 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 1 0 0 0 0 0 0 0 0]);
    m1_6 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 0 1 0 0 0 0 0 0 0]);
    m2_1 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[1 0 0 0 0 0 0 0 0 0 0 0 0]);
    m2_2 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 1 0 0 0 0 0 0 0 0 0 0 0]);
    m2_3 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 1 0 0 0 0 0 0 0 0 0 0]);
    m2_4 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 1 0 0 0 0 0 0 0 0 0]);
    m2_5 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 1 0 0 0 0 0 0 0 0]);
    m2_6 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 0 1 0 0 0 0 0 0 0]);
    m3_1 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[1 0 0 0 0 0 0 0 0 0 0 0 0]);
    m3_2 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 1 0 0 0 0 0 0 0 0 0 0 0]);
    m3_3 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 1 0 0 0 0 0 0 0 0 0 0]);
    m3_4 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 1 0 0 0 0 0 0 0 0 0]);
    m3_5 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 1 0 0 0 0 0 0 0 0]);
    m3_6 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 0 1 0 0 0 0 0 0 0]);
    m4_1 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[1 0 0 0 0 0 0 0 0 0 0 0 0]);
    m4_2 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 1 0 0 0 0 0 0 0 0 0 0 0]);
    m4_3 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 1 0 0 0 0 0 0 0 0 0 0]);
    m4_4 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 1 0 0 0 0 0 0 0 0 0]);
    m4_5 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 1 0 0 0 0 0 0 0 0]);
    m4_6 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 0 1 0 0 0 0 0 0 0]);
    m5_1 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[1 0 0 0 0 0 0 0 0 0 0 0 0]);
    m5_2 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 1 0 0 0 0 0 0 0 0 0 0 0]);
    m5_3 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 1 0 0 0 0 0 0 0 0 0 0]);
    m5_4 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 1 0 0 0 0 0 0 0 0 0]);
    m5_5 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 1 0 0 0 0 0 0 0 0]);
    m5_6 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 0 1 0 0 0 0 0 0 0]);
    m6_1 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[1 0 0 0 0 0 0 0 0 0 0 0 0]);
    m6_2 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 1 0 0 0 0 0 0 0 0 0 0 0]);
    m6_3 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 1 0 0 0 0 0 0 0 0 0 0]);
    m6_4 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 1 0 0 0 0 0 0 0 0 0]);
    m6_5 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 1 0 0 0 0 0 0 0 0]);
    m6_6 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6) -g0(3)],[0 0 0 0 0 1 0 0 0 0 0 0 0]);
    
    
    M = [m1_1,m1_2,m1_3,m1_4,m1_5,m1_6;...
         m2_1,m2_2,m2_3,m2_4,m2_5,m2_6;...
         m3_1,m3_2,m3_3,m3_4,m3_5,m3_6;...
         m4_1,m4_2,m4_3,m4_4,m4_5,m4_6;...
         m5_1,m5_2,m5_3,m5_4,m5_5,m5_6;...
         m6_1,m6_2,m6_3,m6_4,m6_5,m6_6];
    
    ga1 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6)],[0 0 0 0 0 0 0 0 0 0 0 0]);
    ga2 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6)],[0 0 0 0 0 0 0 0 0 0 0 0]);
    ga3 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6)],[0 0 0 0 0 0 0 0 0 0 0 0]);
    ga4 = subs(t4dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6)],[0 0 0 0 0 0 0 0 0 0 0 0]);
    ga5 = subs(t5dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6)],[0 0 0 0 0 0 0 0 0 0 0 0]);
    ga6 = subs(t6dyn,[ddq(1) ddq(2) ddq(3) ddq(4) ddq(5) ddq(6) dq(1) dq(2) dq(3) dq(4) dq(5) dq(6)],[0 0 0 0 0 0 0 0 0 0 0 0]);
    
    G = [ga1 ga2 ga3 ga4 ga5 ga6]';
    
    C0Dq = simplify([t1dyn;t2dyn;t3dyn;t4dyn;t5dyn;t6dyn] - M*ddq - G);
    
    Dyn.M = M;
    Dyn.G = G;
    Dyn.C0Dq = C0Dq;
    Dyn.Tau = Tau;
    
end