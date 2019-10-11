clc
clear
close ALL
[q,d_q,d_d_q,m_torque,time,omega,d_omega,alpha] = Validation_Data_Preparation();
f7 = [0;0;0];
t7 = [0;0;0];
lb = zeros(1,96);
lb(1,1:36) = -5;
lb(1,37:90) = -5;
lb(1,91:96) = 0;
ub = zeros(1,96);
ub(1,1:36) = 5;
ub(1,37:90) = 5;
ub(1,91:96) = 10;
FitnessFunction = @(theta) Fitness_Function(theta,q,omega,d_omega,alpha,f7,t7,m_torque);
numberOfVariables = 96;

initial_pop = zeros(100,96);

initial_pop1 = [0 0 0.0982 0 0 0.0599 0 0.1255 0.093 0 0.1065 0 0 -0.0415 0 0 0 0.04125 0 -0.152 0 ...
0 -0.244 0 0 -0.213 0 0 0.083 0 0 -0.083 0 0 0 0.082 0.0048 0 0 0 0.0048 0 0 0 0.0020 0.0058 ...
0 0 0 -0.0034 0 0 0 0.0058 0.0008 0 0 0 0.0066 0 0 0 0.0066 0.0003 0 0 0 0.0032 0 0 0 0.0032 ...
0.0007 0 0 0 0.0004 0 0 0 0.0007 0.0003 0 0 0 -0.0002 0 0 0 0.0003 2 3.42 1.26 0.8 0.8 0.35];

initial_pop(1,:) = initial_pop1;
random = rand(96,1);

for i =2:100
    random = rand(96,1);
    initial_pop(i,:) = initial_pop1 + (random' -0.5)*2/100;
end

options = optimoptions('ga','InitialPopulationMatrix',initial_pop,'FunctionTolerance',1e-6,'PopulationSize',30,'FitnessLimit',1,'MaxGenerations',100*numberOfVariables);
tic
[theta,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub,[],options);
toc
torque = Dynamic_Model(theta,q,omega,d_omega,alpha,f7,t7);

plotter;