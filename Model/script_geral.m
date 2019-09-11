clc
clear
[q,d_q,d_d_q,m_torque,time,omega,d_omega,alpha] = Validation_Data_Preparation();
f7 = [0;0;0];
t7 = [0;0;0];
lb = zeros(1,96);
lb(1,1:36) = -0.1;
lb(1,37:90) = 0;
lb(1,91:96) = 0;
ub = zeros(1,96);
ub(1,1:36) = 0.1;
ub(1,37:90) = 1;
ub(1,91:96) = 1;
FitnessFunction = @(theta) Function_Model_Teste(theta,q,omega,d_omega,alpha,f7,t7,m_torque');
numberOfVariables = 96;
options = optimoptions(@ga,'FunctionTolerance',1e-12);
tic
[theta,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub);
toc

for i =1:length(q)-1
    torque(i,1:6) = Function_Model(theta,q(i,1:6),omega(1:18,i),d_omega(1:18,i),alpha(1:18,i),f7,t7);
end