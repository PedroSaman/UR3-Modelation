function [Tau] = DOF1()
    
    
    q = sym('q',[1 1],'real');
    dq = sym('dq',[1 1],'real');
    ddq = sym('ddq',[1 1],'real');
    
    z0 = [0;0;1];
    
    R0_1 = [cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*round([1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    
    b1 = simplify(R0_1'*z0);
    
    omega1 = b1*dq(1);
    alpha1 = b1*ddq(1) + cross(omega1,b1*dq(1));
    
    g0 = [0;0;-sym('g','real')];
    lc1 = [0;sym('lc1','real');0]; %Distancia entre o frame 0 e o centro de massa 1 em relação ao frame 1
    I1 = [sym('I1x','real'),0,0;0,sym('I1y','real'),0;0,0,sym('I1z','real')];% Considerando apenas a diag principal
    m1 = sym('m1','real');
    beta1 = sym('beta1','real');
    
    a_c1 = cross(alpha1,lc1) + cross(omega1,cross(omega1,lc1));
    
    g1 = round(R0_1')*g0;
    f1 = m1*a_c1 - m1*g1;
    t1 = - cross(f1,lc1) + cross(omega1,I1*omega1) + I1*alpha1 + beta1*omega1; % Torque em relação ao eixo 1
    t1dyn = simplify(b1'*t1);
    
    Tau = t1dyn;

end