function [theta1,theta2,theta3,theta4,theta5,theta6,FIT] = Identification6DOF()
% MSQ identification    
    [input] = Identification_data_Loader(); %Id dados reais
    %load 'simulated_data.mat' input; %Id de simulação
    [psi1,psi2,psi3,psi4,psi5,psi6] = PsiMatrix(input);
    
    l = length(input.tout);
    
    %Training section
    theta1 = pinv(psi1(1:0.7*l,:))*input.tau1(1:0.7*l);
    theta2 = pinv(psi2(1:0.7*l,:))*input.tau2(1:0.7*l);
    theta3 = pinv(psi3(1:0.7*l,:))*input.tau3(1:0.7*l);
    theta4 = pinv(psi4(1:0.7*l,:))*input.tau4(1:0.7*l);
    theta5 = pinv(psi5(1:0.7*l,:))*input.tau5(1:0.7*l);
    theta6 = pinv(psi6(1:0.7*l,:))*input.tau6(1:0.7*l);
    clc

    %Validation section
    etau1 = psi1(0.7*l:end,:)*theta1;
    etau2 = psi2(0.7*l:end,:)*theta2;
    etau3 = psi3(0.7*l:end,:)*theta3;
    etau4 = psi4(0.7*l:end,:)*theta4;
    etau5 = psi5(0.7*l:end,:)*theta5;
    etau6 = psi6(0.7*l:end,:)*theta6;
    
    %Plotting
    plot(input.tout(0.7*l:end,:),etau1);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau1(0.7*l:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)') 
    figure
    plot(input.tout(0.7*l:end,:),etau2);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau2(0.7*l:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau3);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau3(0.7*l:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau4);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau4(0.7*l:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau5);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau5(0.7*l:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau6);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau6(0.7*l:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    
    %Curve Fitness
    FIT = fitness([etau1,etau2,etau3,etau4,etau5,etau6],[input.tau1(0.7*l:end,:),input.tau2(0.7*l:end,:)...
        ,input.tau3(0.7*l:end,:),input.tau4(0.7*l:end,:),input.tau5(0.7*l:end,:),input.tau6(0.7*l:end,:)])
end