function [Tau] = DOF2()
    
    q = sym('q',[2 1],'real');
    dq = sym('dq',[2 1],'real');
    ddq = sym('ddq',[2 1],'real');
    
    z0 = [0;0;1];
    
    R0_1 = [cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R1_2 = [cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1];
    R0_2 = R0_1*R1_2;
    
    % variáveis estáticas para simplificar as expressões abaixo
    b1 = simplify(R0_1'*z0);
    b2 = simplify(R0_2'*R0_1*z0);
    
    omega1 = b1*dq(1);
    omega2 = R1_2'*omega1 + b2*dq(2);
    
    alpha1 = b1*ddq(1) + cross(omega1,b1*dq(1));
    alpha2 = R1_2'*alpha1 + b2*ddq(2) + cross(omega2,b2*dq(2));
     
    g0 = [0;0;-sym('g','real')];
    lc1 = [0;sym('lc1y','real');0]; %Distancia entre o frame 0 e o centro de massa 1 em relação ao frame 1
    lc2 = [-sym('lc2x','real');0;sym('lc2z','real')];
    l1 = [0;sym('l1y','real');0];
    rc1_1 = lc1 - l1;
    I1 = [sym('I1x','real'),0,0;0,sym('I1y','real'),0;0,0,sym('I1z','real')];% Considerando apenas a diag principal
    I2 = [sym('I2x','real'),0,0;0,sym('I2y','real'),0;0,0,sym('I2z','real')];% Considerando apenas a diag principal
    m1 = sym('m1','real');
    m2 = sym('m2','real');
    beta1 = sym('beta1','real');
    beta2 = sym('beta2','real');
     
    a_c1 = cross(alpha1,lc1) + cross(omega1,cross(omega1,lc1));
    a_e1 = cross(alpha1,l1) + cross(omega1,cross(omega1,l1));
    
    a_c2 = R1_2'*a_e1 + cross(alpha2,lc2) + cross(omega2,cross(omega2,lc2));
    
    g2 = R0_2'*g0;
    f2 = m2*a_c2 - m2*g2;
    t2 = - cross(f2,lc2) + cross(omega2,I2*omega2) + I2*alpha2 + beta2*omega2;
    t2dyn = simplify(b2'*t2);
    
    g1 = R0_1'*g0;
    f1 = R1_2*f2 + m1*a_c1 - m1*g1;
    t1 = R1_2*t2 - cross(f1,lc1) + cross(R1_2*f2,rc1_1) + cross(omega1,I1*omega1) + I1*alpha1 + beta1*omega1; % Torque em relação ao eixo 1
    t1dyn = simplify(b1'*t1);
     
    Tau = [t1dyn;t2dyn];
end