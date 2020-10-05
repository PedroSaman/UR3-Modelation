function [theta1,theta2,theta3,theta4,theta5,theta6,FIT] = Identification6DOF()
% MSQ identification    
    [input] = Identification_data_Loader(); %Real data
    [psi1,psi2,psi3,psi4,psi5,psi6] = PsiMatrix(input);
    
    l = length(input.tout);
    
%     Training section
%      theta1 = pinv(psi1(1:0.7*l,:))*input.tau1(1:0.7*l);
%      theta2 = pinv(psi2(1:0.7*l,:))*input.tau2(1:0.7*l);
%      theta3 = pinv(psi3(1:0.7*l,:))*input.tau3(1:0.7*l);
%      theta4 = pinv(psi4(1:0.7*l,:))*input.tau4(1:0.7*l);
%      theta5 = pinv(psi5(1:0.7*l,:))*input.tau5(1:0.7*l);
%      theta6 = pinv(psi6(1:0.7*l,:))*input.tau6(1:0.7*l);
%     clc
     theta6 = lsqlin(psi6(1:0.7*l,:),input.tau6(1:0.7*l),[],[],[],[],[-inf,-inf,-inf]);%,[0,inf,inf]);
     theta5 = lsqlin(psi5(1:0.7*l,:),input.tau5(1:0.7*l),[],[],[],[],[0,0,0,0,0,0,0,0,0,0,0,0,0,theta6(2)],[inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,inf,theta6(2)]);%[0,-inf,-inf,-inf,-inf,-inf,-inf,-inf,0,0,0,theta6(2),0,0]);%,[inf,inf,inf,inf,inf,inf,inf,0,inf,inf,inf,inf,inf,inf]);
     theta4 = lsqlin(psi4(1:0.7*l,:),input.tau4(1:0.7*l),[],[],[],[],[-inf,-inf,-inf,theta5(7),(theta5(6)-theta5(4)),-inf,-inf,theta5(3),theta5(5),theta5(1),theta5(2),-inf,-inf,-inf,theta5(9),theta5(10),theta5(11),theta5(12),theta5(13),theta6(2)],[inf,inf,inf,theta5(7),(theta5(6)-theta5(4)),inf,inf,theta5(3),theta5(5),theta5(1),theta5(2),inf,inf,inf,theta5(9),theta5(10),theta5(11),theta5(12),theta5(13),theta6(2)]);
     theta3 = lsqlin(psi3(1:0.7*l,:),input.tau3(1:0.7*l),[],[],[],[],[theta5(7),theta4(3),-inf,theta4(2),theta4(1),-inf,-inf,theta5(1),(theta5(4)-theta5(6)),theta5(3),theta4(6),theta5(2),theta5(5),-inf,theta4(7),-inf,-inf,-inf,-theta4(13),theta4(14),theta5(9),theta5(10),theta5(11),theta5(12),theta5(13),theta6(2)],[theta5(7),theta4(3),inf,theta4(2),theta4(1),inf,inf,theta5(1),(theta5(4)-theta5(6)),theta5(3),theta4(6),theta5(2),theta5(5),inf,theta4(7),inf,inf,inf,-theta4(13),theta4(14),theta5(9),theta5(10),theta5(11),theta5(12),theta5(13),theta6(2)]);
     theta2 = lsqlin(psi2(1:0.7*l,:),input.tau2(1:0.7*l),[],[],[],[],[theta3(7),theta3(3),-inf,(theta5(6)-theta5(4)),theta3(6),theta4(3),-inf,theta4(2),theta4(7),theta4(1),-inf,theta5(7),theta5(5),theta3(14),theta4(6),theta5(1),theta5(2),theta5(3),-inf,-inf,-inf,-theta3(17),theta3(18),theta4(13),theta4(14),theta5(9),theta5(10),theta5(11),theta5(12),theta5(13),theta6(2)],[theta3(7),theta3(3),inf,(theta5(6)-theta5(4)),theta3(6),theta4(3),inf,theta4(2),theta4(7),theta4(1),inf,theta5(7),theta5(5),theta3(14),theta4(6),theta5(1),theta5(2),theta5(3),inf,inf,inf,-theta3(17),theta3(18),theta4(13),theta4(14),theta5(9),theta5(10),theta5(11),theta5(12),theta5(13),theta6(2)]);
     theta1 = lsqlin(psi1(1:0.7*l,:),input.tau1(1:0.7*l),[],[],[],[],[theta5(5),theta5(3),theta5(2),(theta5(4)-theta5(6)),theta5(1),theta2(3),-inf,-inf,theta4(7),theta4(6),theta3(14),-inf,theta4(1),theta3(6),-inf,-inf,-inf,-inf,-inf,-inf,-inf,-inf,theta5(9),theta5(10),theta5(11),theta6(2),theta5(13),theta5(12)],[theta5(5),theta5(3),theta5(2),(theta5(4)-theta5(6)),theta5(1),theta2(3),inf,inf,theta4(7),theta4(6),theta3(14),inf,theta4(1),theta3(6),inf,inf,inf,inf,inf,inf,inf,inf,theta5(9),theta5(10),theta5(11),theta6(2),theta5(13),theta5(12)]);
     clc
   
    %Validation section
    
    etau6 = psi6(0.7*l:end,:)*theta6;
    etau5 = psi5(0.7*l:end,:)*theta5;
    etau4 = psi4(0.7*l:end,:)*theta4;
    etau3 = psi3(0.7*l:end,:)*theta3;
    etau2 = psi2(0.7*l:end,:)*theta2;
    etau1 = psi1(0.7*l:end,:)*theta1;
        
    %Plotting
    plot(input.tout(0.7*l:end,:),etau1);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau1(0.7*l:end,:));
    grid on
    title('Joint1 Torque');
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)') 
    figure
    plot(input.tout(0.7*l:end,:),etau2);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau2(0.7*l:end,:));
    grid on
    title('Joint2 Torque');
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau3);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau3(0.7*l:end,:));
    grid on
    title('Joint3 Torque');
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau4);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau4(0.7*l:end,:));
    grid on
    title('Joint4 Torque');
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau5);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau5(0.7*l:end,:));
    grid on
    title('Joint5 Torque');
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l:end,:),etau6);
    hold on
    plot(input.tout(0.7*l:end,:),input.tau6(0.7*l:end,:));
    grid on
    title('Joint6 Torque');
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    clc
    %Curve Fitness
      FIT = fitness([etau1,etau2,etau3,etau4,etau5,etau6],[input.tau1(0.7*l:end,:),input.tau2(0.7*l:end,:)...
        ,input.tau3(0.7*l:end,:),input.tau4(0.7*l:end,:),input.tau5(0.7*l:end,:),input.tau6(0.7*l:end,:)])
end