function [Dyn] = DOF3()
    
    q = sym('q',[3 1],'real');
    dq = sym('dq',[3 1],'real');
    ddq = sym('ddq',[3 1],'real');
    
    z0 = [0;0;1];
    
    R0_1 = [cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R1_2 = [cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1];
    R2_3 = [cos(q(3)) -sin(q(3)) 0;sin(q(3)) cos(q(3)) 0; 0 0 1];
    R0_2 = R0_1*R1_2;
    R0_3 = R0_2*R2_3;
    
    % variáveis estáticas para simplificar as expressões abaixo
    b1 = simplify(R0_1'*z0);
    b2 = simplify(R0_2'*R0_1*z0);
    b3 = simplify(R0_3'*R0_2*z0);
    
    omega1 = b1*dq(1);
    omega2 = R1_2'*omega1 + b2*dq(2);
    omega3 = R2_3'*omega2 + b3*dq(3);
    
    alpha1 = b1*ddq(1) + cross(omega1,b1*dq(1));
    alpha2 = R1_2'*alpha1 + b2*ddq(2) + cross(omega2,b2*dq(2));
    alpha3 = R2_3'*alpha2 + b3*ddq(3) + cross(omega3,b3*dq(3));
    
    g0 = [0;0;-sym('g','real')];
    lc1 = [0;sym('lc1y','real');0]; %Distancia entre o frame 0 e o centro de massa 1 em relação ao frame 1
    lc2 = [-sym('lc2x','real');0;sym('lc2z','real')];
    lc3 = [-sym('lc3x','real');0;-sym('lc3z','real')];
    l1 = [0;sym('l1y','real');0];
    l2 = [-sym('l2x','real');0;sym('l2z','real')];
    rc1_1 = lc1 - l1;
    rc2_2 = lc2 - l2;
    I1 = [sym('I1x','real'),0,0;0,sym('I1y','real'),0;0,0,sym('I1z','real')];% Considerando apenas a diag principal
    I2 = [sym('I2x','real'),0,0;0,sym('I2y','real'),0;0,0,sym('I2z','real')];% Considerando apenas a diag principal
    I3 = [sym('I3x','real'),0,0;0,sym('I3y','real'),0;0,0,sym('I3z','real')];% Considerando apenas a diag principal
    m1 = sym('m1','real');
    m2 = sym('m2','real');
    m3 = sym('m3','real');
    beta1 = sym('beta1','real');
    beta2 = sym('beta2','real');
    beta3 = sym('beta3','real');
     
    a_c1 = cross(alpha1,lc1) + cross(omega1,cross(omega1,lc1));
    a_e1 = cross(alpha1,l1) + cross(omega1,cross(omega1,l1));
    
    a_c2 = R1_2'*a_e1 + cross(alpha2,lc2) + cross(omega2,cross(omega2,lc2));
    a_e2 = R1_2'*a_e1 + cross(alpha2,l2) + cross(omega2,cross(omega2,l2));
     
    a_c3 = R2_3'*a_e2 + cross(alpha3,lc3) + cross(omega3,cross(omega3,lc3));
    
    g3 = R0_3'*g0;
    f3 = m3*a_c3 - m3*g3;
    t3 = - cross(f3,lc3) + cross(omega3,I3*omega3) + I3*alpha3 + beta3*omega3;
    t3dyn = simplify(b3'*t3);
    
    g2 = R0_2'*g0;
    f2 = R2_3*f3 + m2*a_c2 - m2*g2;
    t2 = R2_3*t3 - cross(f2,lc2) + cross(R2_3*f3,rc2_2) + cross(omega2,I2*omega2) + I2*alpha2 + beta2*omega2;
    t2dyn = simplify(b2'*t2);
    
    g1 = R0_1'*g0;
    f1 = R1_2*f2 + m1*a_c1 - m1*g1;
    t1 = R1_2*t2 - cross(f1,lc1) + cross(R1_2*f2,rc1_1) + cross(omega1,I1*omega1) + I1*alpha1 + beta1*omega1; % Torque em relação ao eixo 1
    t1dyn = simplify(b1'*t1);
     
    Tau = [t1dyn;t2dyn;t3dyn];
    
    m1_1 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[1 0 0 0 0 0 0]);
    m1_2 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[0 1 0 0 0 0 0]);
    m1_3 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[0 0 1 0 0 0 0]);
    m2_1 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[1 0 0 0 0 0 0]);
    m2_2 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[0 1 0 0 0 0 0]);
    m2_3 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[0 0 1 0 0 0 0]);
    m3_1 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[1 0 0 0 0 0 0]);
    m3_2 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[0 1 0 0 0 0 0]);
    m3_3 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3) -g0(3)],[0 0 1 0 0 0 0]);
    
    M = [m1_1,m1_2,m1_3;m2_1,m2_2,m2_3;m3_1,m3_2,m3_3];
    
    ga1 = subs(t1dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3)],[0 0 0 0 0 0]);
    ga2 = subs(t2dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3)],[0 0 0 0 0 0]);
    ga3 = subs(t3dyn,[ddq(1) ddq(2) ddq(3) dq(1) dq(2) dq(3)],[0 0 0 0 0 0]);
    
    G = [ga1;ga2;ga3];
    
    C0Dq = [t1dyn;t2dyn;t3dyn] - M*ddq - G;
    
    Dyn.M = M;
    Dyn.G = G;
    Dyn.C0Dq = C0Dq;
    Dyn.Tau = Tau;
   
    %teste = M*ddq+C0Dq+G;
    
end