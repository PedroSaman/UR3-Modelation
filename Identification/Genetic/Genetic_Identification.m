clc
clear
close ALL
[q,d_q,d_d_q,m_torque,time,omega,d_omega,alpha] = Data_Preparation();
normalization = std(m_torque);
f7 = [0;0;0];
t7 = [0;0;0];
lb = zeros(1,78);
ub = zeros(1,78);
lb(1,1:36) = -3;
lb(1,37:72) = -2;
lb(1,73:78) = 0;
ub(1,1:36) = 3;
ub(1,37:72) = 2;
ub(1,73:78) = 10;
FitnessFunction = @(theta) Fitness_Function(theta);
numberOfVariables = 78;

initial_pop = zeros(30,78);
initial_pop1 = estimated_parameters();
initial_pop(1,:) = initial_pop1;

%for i =1:30
for i =2:30
    random = rand(78,1);
    initial_pop(i,:) = initial_pop1 + (random' -0.5)*2/50;
end

options = optimoptions('ga','InitialPopulationMatrix',initial_pop,'FunctionTolerance',1e-8,'PopulationSize',30,'FitnessLimit',0.1,'MaxGenerations',100*numberOfVariables);
tic
[theta,fval] = ga(FitnessFunction,numberOfVariables,[],[],[],[],lb,ub,[],options);
toc
torque = Dynamic_Model(theta,q,omega,d_omega,alpha,f7,t7);

plotter;
save 'theta.mat','theta.mat';