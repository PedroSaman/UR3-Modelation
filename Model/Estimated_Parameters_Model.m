clear
clc

[theta,d_theta,d_d_theta,m_torque,time,x,y] = Validation_Data_Preparation();
f7 = [0;0;0];
t7 = [0;0;0];
R6_7 = eye(3);

%Pre alocation
omega_1 = zeros([3,length(time)]);
omega_2 = zeros([3,length(time)]);
omega_3 = zeros([3,length(time)]);
omega_4 = zeros([3,length(time)]);
omega_5 = zeros([3,length(time)]);
omega_6 = zeros([3,length(time)]);

alpha_1 = zeros([3,length(time)]);
alpha_3 = zeros([3,length(time)]);
alpha_2 = zeros([3,length(time)]);
alpha_4 = zeros([3,length(time)]);
alpha_5 = zeros([3,length(time)]);
alpha_6 = zeros([3,length(time)]);

d_omega1 = zeros([3,length(time)-1]);
d_omega2 = zeros([3,length(time)-1]);
d_omega3 = zeros([3,length(time)-1]);
d_omega4 = zeros([3,length(time)-1]);
d_omega5 = zeros([3,length(time)-1]);
d_omega6 = zeros([3,length(time)-1]);

t1dyn = zeros([length(time),1]);
t2dyn = zeros([length(time),1]);
t3dyn = zeros([length(time),1]);
t4dyn = zeros([length(time),1]);
t5dyn = zeros([length(time),1]);
t6dyn = zeros([length(time),1]);

z0 = [0;0;1];
m = ([2,3.42,1.26,0.8,0.8,0.35]);

%Distância entre o frame i-1 e o centro de massa do link i r(i-1)_ci 
%Considerando que o centro de massa dos links está no centro gemométrico
r0_c1 = [0 0 0.0982].';
r1_c2 = [0 0 0.0599].';
r2_c3 = [0 0.1255 0.093].';
r3_c4 = [0 0.1065 0].';
r4_c5 = [0 -0.0415 0].';
r5_c6 = [0 0 0.04125].';

%Distância entre os frames r(i-1)_i
r0_1 = [0 -0.152 0].';
r1_2 = [0 -0.244 0].';
r2_3 = [0 -0.213  0].';
r3_4 = [0 0.083 0].';
r4_5 = [0 -0.083 0].';
r5_6 = [0 0 0.082].';

%Distância entre o centro de massa do link i e o frame i
r1_c1 = r0_c1 - r0_1;
r2_c2 = r1_c2 - r1_2;
r3_c3 = r2_c3 - r2_3;
r4_c4 = r3_c4 - r3_4;
r5_c5 = r4_c5 - r4_5;
r6_c6 = r5_c6 - r5_6;

%Gravity vector
g0 = [0;0;-9.81];

%Matrizes de Inercia dos links aproximadas para um cilindro
I1 = [m(1)*(3*0.0447^2+0.152^2)/12,0,0;0,m(1)*(3*0.0447^2+0.152^2)/12,0;0,0,0.5*m(1)*0.0447^2];%sym('I1_', [3 3]);
I2 = [m(2)*(3*0.0443^2+0.120^2)/12,0,0;0,-0.5*m(2)*0.0443^2,0;0,0,m(2)*(3*0.0443^2+0.120^2)/12];%sym('I2_', [3 3]);
I3 = [0.5*m(3)*0.035^2,0,0;0,m(3)*(3*0.035^2+0.244^2)/12,0;0,0,m(3)*(3*0.035^2+0.244^2)/12];%sym('I3_', [3 3]);
I4 = [0.5*m(4)*0.0295^2,0,0;0,m(4)*(3*0.0295^2+0.213^2)/12,0;0,0,m(4)*(3*0.0295^2+0.213^2)/12];%sym('I4_', [3 3]);
I5 = [m(5)*(3*0.03145^2+0.083^2)/12,0,0;0,0.5*m(5)*0.03145^2,0;0,0,m(5)*(3*0.03145^2+0.083^2)/12];%sym('I5_', [3 3]);
I6 = [m(6)*(3*0.0316^2+0.082^2)/12,0,0;0,-0.5*m(6)*0.0316^2,0;0,0,m(6)*(3*0.0316^2+0.082^2)/12];%sym('I6_', [3 3]);

%Recursão para frente
for i = 1:length(time)-1
    %Matrizes de Rotação
    R0_1 = ([cos(theta(i,1)+pi/2) -sin(theta(i,1)+pi/2) 0;sin(theta(i,1)+pi/2) cos(theta(i,1)+pi/2) 0; 0 0 1]*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R1_2 = ([cos(theta(i,2)-pi/2) -sin(theta(i,2)-pi/2) 0;sin(theta(i,2)-pi/2) cos(theta(i,2)-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R2_3 = ([cos(theta(i,3)) -sin(theta(i,3)) 0;sin(theta(i,3)) cos(theta(i,3)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R3_4 = ([cos(theta(i,4)-pi/2) -sin(theta(i,4)-pi/2) 0;sin(theta(i,4)-pi/2) cos(theta(i,4)-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R4_5 = ([cos(theta(i,5)) -sin(theta(i,5)) 0;sin(theta(i,5)) cos(theta(i,5)) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R5_6 = ([cos(theta(i,6)) -sin(theta(i,6)) 0;sin(theta(i,6)) cos(theta(i,6)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R0_2 = R0_1*R1_2;
    R0_3 = R0_2*R2_3;
    R0_4 = R0_3*R3_4;
    R0_5 = R0_4*R4_5;
    R0_6 = R0_5*R5_6;
    
    %Parâmetro B
    b1 = R0_1.'*z0;     %[0;-1;0];
    b2 = R0_2.'*R0_1*z0;%[0;0;1];
    b3 = R0_3.'*R0_2*z0;%[0;0;1];
    b4 = R0_4.'*R0_3*z0;%[0;-1;0];
    b5 = R0_5.'*R0_4*z0;%[0;1;0];
    b6 = R0_6.'*R0_5*z0;%[0;0;1];
    
    omega_1(1:3,i) = b1*d_theta(i,1);
    alpha_1(1:3,i) = b1*d_d_theta(i,1) + cross(omega_1(1:3,i),(b1*d_theta(i,1)));
    
    omega_2(1:3,i) = R1_2.'*omega_1(1:3,i) + b2*d_theta(i,2);
    alpha_2(1:3,i) = R1_2.'*alpha_1(1:3,i) + b2*d_d_theta(i,2) + cross(omega_2(1:3,i),(b2*d_theta(i,2)));
    
    omega_3(1:3,i) = R2_3.'*omega_2(1:3,i) + b3*d_theta(i,3);
    alpha_3(1:3,i) = R2_3.'*alpha_2(1:3,i) + b3*d_d_theta(i,3) + cross(omega_3(1:3,i),(b3*d_theta(i,3)));
    
    omega_4(1:3,i) = R3_4.'*omega_3(1:3,i) + b4*d_theta(i,4);
    alpha_4(1:3,i) = R3_4.'*alpha_3(1:3,i) + b4*d_d_theta(i,4) + cross(omega_4(1:3,i),(b4*d_theta(i,4)));
    
    omega_5(1:3,i) = R4_5.'*omega_4(1:3,i) + b5*d_theta(i,5);
    alpha_5(1:3,i) = R4_5.'*alpha_4(1:3,i) + b5*d_d_theta(i,5) + cross(omega_5(1:3,i),(b5*d_theta(i,5)));
    
    omega_6(1:3,i) = R5_6.'*omega_5(1:3,i) + b6*d_theta(i,6);
    alpha_6(1:3,i) = R5_6.'*alpha_5(1:3,i) + b6*d_d_theta(i,6) + cross(omega_6(1:3,i),(b6*d_theta(i,2)));
    
end

time2 = diff(time);

for i=1:length(time)-1
    d_omega1 = (diff(omega_1.')/time2(i)).';
    d_omega2 = (diff(omega_2.')/time2(i)).';
    d_omega3 = (diff(omega_3.')/time2(i)).';
    d_omega4 = (diff(omega_4.')/time2(i)).';
    d_omega5 = (diff(omega_5.')/time2(i)).';
    d_omega6 = (diff(omega_6.')/time2(i)).';
end

for i=1:length(time)-1
    
    %Matrizes de Rotação
    R0_1 = ([cos(theta(i,1)+pi/2) -sin(theta(i,1)+pi/2) 0;sin(theta(i,1)+pi/2) cos(theta(i,1)+pi/2) 0; 0 0 1]*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R1_2 = ([cos(theta(i,2)-pi/2) -sin(theta(i,2)-pi/2) 0;sin(theta(i,2)-pi/2) cos(theta(i,2)-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R2_3 = ([cos(theta(i,3)) -sin(theta(i,3)) 0;sin(theta(i,3)) cos(theta(i,3)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R3_4 = ([cos(theta(i,4)-pi/2) -sin(theta(i,4)-pi/2) 0;sin(theta(i,4)-pi/2) cos(theta(i,4)-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(pi/2)]);
    R4_5 = ([cos(theta(i,5)) -sin(theta(i,5)) 0;sin(theta(i,5)) cos(theta(i,5)) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R5_6 = ([cos(theta(i,6)) -sin(theta(i,6)) 0;sin(theta(i,6)) cos(theta(i,6)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R0_2 = R0_1*R1_2;
    R0_3 = R0_2*R2_3;
    R0_4 = R0_3*R3_4;
    R0_5 = R0_4*R4_5;
    R0_6 = R0_5*R5_6;
    
    %Parâmetro B
    b1 = R0_1.'*z0;     %[0;-1;0];
    b2 = R0_2.'*R0_1*z0;%[0;0;1];
    b3 = R0_3.'*R0_2*z0;%[0;0;1];
    b4 = R0_4.'*R0_3*z0;%[0;-1;0];
    b5 = R0_5.'*R0_4*z0;%[0;1;0];
    b6 = R0_6.'*R0_5*z0;%[0;0;1];
    
    a_e_1 = cross(d_omega1(1:3,i),r0_1) + cross(omega_1(1:3,i),(cross(omega_1(1:3,i),r0_1)));
    a_c_1 = cross(d_omega1(1:3,i),r0_c1) + cross(omega_1(1:3,i),(cross(omega_1(1:3,i),r0_c1)));
    
    a_e_2 = R1_2.'*a_e_1 + cross(d_omega2(1:3,i),r1_2) + cross(omega_2(1:3, i),(cross(omega_2(1:3,i),r1_2)));
    a_c_2 = R1_2.'*a_e_1 + cross(d_omega2(1:3,i),r1_c2) + cross(omega_2(1:3, i),(cross(omega_2(1:3,i),r1_c2)));
    
    a_e_3 = R2_3.'*a_e_2 + cross(d_omega3(1:3,i),r2_3) + cross(omega_3(1:3,i),(cross(omega_3(1:3,i),r2_3)));
    a_c_3 = R2_3.'*a_e_2 + cross(d_omega3(1:3,i),r2_c3) + cross(omega_3(1:3,i),(cross(omega_3(1:3,i),r2_c3)));
    
    a_e_4 = R3_4.'*a_e_3 + cross(d_omega4(1:3,i),r3_4) + cross(omega_4(1:3,i),(cross(omega_4(1:3,i),r3_4)));
    a_c_4 = R3_4.'*a_e_3 + cross(d_omega4(1:3,i),r3_c4) + cross(omega_4(1:3,i),(cross(omega_4(1:3,i),r3_c4)));
    
    a_e_5 = R4_5.'*a_e_4 + cross(d_omega5(1:3,i),r4_5) + cross(omega_5(1:3,i),(cross(omega_5(1:3,i),r4_5)));
    a_c_5 = R4_5.'*a_e_4 + cross(d_omega5(1:3,i),r4_c5) + cross(omega_5(1:3,i),(cross(omega_5(1:3,i),r4_c5)));
    
    a_c_6 = R5_6.'*a_e_5 + cross(d_omega6(1:3,i),r5_c6) + cross(omega_6(1:3,i),(cross(omega_6(1:3,i),r5_c6)));
    
    g6 = R0_6.'*g0;
    f6 = R6_7*f7 + m(6)*a_c_6 - m(6)*g6;
    t6 = R6_7*t7 - cross(f6,r5_c6) + cross(omega_6(1:3,i),(I6*omega_6(1:3,i)) + I6*alpha_6(1:3,i));
    t6dyn(i) = (b6.'*t6);
    
    g5 = R0_5.'*g0;
    f5 = R5_6*f6 + m(5)*a_c_5 - m(5)*g5;
    t5 = R5_6*t6 - cross(f5,r4_c5) + cross((R5_6*f6),r5_c5) + cross(omega_5(1:3,i),(I5*omega_5(1:3,i))) + I5*alpha_5(1:3,i);
    t5dyn(i) = (b5.'*t5);
    
    g4 = R0_4.'*g0;
    f4 = R4_5*f5 + m(4)*a_c_4 - m(4)*g4;
    t4 = R4_5*t5 - cross(f4,r3_c4) + cross((R4_5*f5),r4_c4) + cross(omega_4(1:3,i),(I4*omega_4(1:3,i))) + I4*alpha_4(1:3,i);
    t4dyn(i) = (b4.'*t4);
    
    g3 = R0_3.'*g0;
    f3 = R3_4*f4 + m(3)*a_c_3 - m(3)*g3;
    t3 = R3_4*t4 - cross(f3,r2_c3) + cross((R3_4*f4),r3_c3) + cross(omega_3(1:3,i),(I3*omega_3(1:3,i))) + I3*alpha_3(1:3,i);
    t3dyn(i) = (b3.'*t3);
    
    g2 = R0_2.'*g0;
    f2 = R2_3*f3 + m(2)*a_c_2 - m(2)*g2;
    t2 = R2_3*t3 - cross(f2,r1_c2) + cross((R2_3*f3),r2_c2) + cross(omega_2(1:3,i),(I2*omega_2(1:3,i))) + I2*alpha_2(1:3,i);
    t2dyn(i) = (b2.'*t2);
    
    g1 = R0_1.'*g0;
    f1 = R1_2*f2 + m(1)*a_c_1 - m(1)*g1;
    t1 = R1_2*t2 - cross(f1,r0_c1) + cross((R1_2*f2),r1_c1) + cross(omega_1(1:3,i),(I1*omega_1(1:3,i))) + I1*alpha_1(1:3,i);
    t1dyn(i) = (b1.'*t1);
end

torque = [t1dyn.'; t2dyn.'; t3dyn.'; t4dyn.'; t5dyn.'; t6dyn.'].';