function [theta1,theta2,theta3,theta4,theta5,theta6,FIT] = RLS()
%Recursive LeastSquare method need to chose lambda etc
    
    [input] = Identification_data_Loader();
    [psi1,psi2,psi3,psi4,psi5,psi6] = PsiMatrix(input);
    l = length(input.tout);
    
    %theta1
    P=eye(28)*10^6;
    lambda=0.999;
    %lambda=1;
    teta_v(:,1)=zeros(1,28);
    for k=2:0.7*l
        psi_k=psi1(k,:)';
        K_k = (P*psi_k)/(psi_k'*P*psi_k+lambda);
        teta_v(:,k)=teta_v(:,k-1)+K_k*(input.tau1(k)-psi_k'*teta_v(:,k-1));
        P=(P-(P*(psi_k*psi_k')*P)/(psi_k'*P*psi_k+lambda))/lambda;
    end
    
    theta1 = teta_v(:,end);
    clear P teta_v
    
    %theta2
    P=eye(31)*10^6;
    lambda=0.9979;
    %lambda=1;
    teta_v(:,1)=zeros(1,31);
    for k=2:0.7*l
        psi_k=psi2(k,:)';
        K_k = (P*psi_k)/(psi_k'*P*psi_k+lambda);
        teta_v(:,k)=teta_v(:,k-1)+K_k*(input.tau2(k)-psi_k'*teta_v(:,k-1));
        P=(P-(P*(psi_k*psi_k')*P)/(psi_k'*P*psi_k+lambda))/lambda;
    end
    
    theta2 = teta_v(:,end);
    clear P teta_v
    
    %theta3
    P=eye(26)*10^6;
    lambda=0.9979;
    %lambda=1;
    teta_v(:,1)=zeros(1,26);
    for k=2:0.7*l
        psi_k=psi3(k,:)';
        K_k = (P*psi_k)/(psi_k'*P*psi_k+lambda);
        teta_v(:,k)=teta_v(:,k-1)+K_k*(input.tau3(k)-psi_k'*teta_v(:,k-1));
        P=(P-(P*(psi_k*psi_k')*P)/(psi_k'*P*psi_k+lambda))/lambda;
    end
    
    theta3 = teta_v(:,end);
    clear P teta_v
    
    %theta4
    P=eye(20)*10^6;
    lambda=0.999;
    %lambda=1;
    teta_v(:,1)=zeros(1,20);
    for k=2:0.7*l
        psi_k=psi4(k,:)';
        K_k = (P*psi_k)/(psi_k'*P*psi_k+lambda);
        teta_v(:,k)=teta_v(:,k-1)+K_k*(input.tau4(k)-psi_k'*teta_v(:,k-1));
        P=(P-(P*(psi_k*psi_k')*P)/(psi_k'*P*psi_k+lambda))/lambda;
    end
    
    theta4 = teta_v(:,end);
    clear P teta_v
    
    %theta5
    P=eye(14)*10^6;
    lambda=0.999;
    %lambda=1;
    teta_v(:,1)=zeros(1,14);
    for k=2:0.7*l
        psi_k=psi5(k,:)';
        K_k = (P*psi_k)/(psi_k'*P*psi_k+lambda);
        teta_v(:,k)=teta_v(:,k-1)+K_k*(input.tau5(k)-psi_k'*teta_v(:,k-1));
        P=(P-(P*(psi_k*psi_k')*P)/(psi_k'*P*psi_k+lambda))/lambda;
    end
    
    theta5 = teta_v(:,end);
    clear P teta_v
    
    %theta6
    P=eye(3)*10^6;
    lambda=0.999;
    %lambda=1;
    teta_v(:,1)=zeros(1,3);
    for k=2:0.7*l
        psi_k=psi6(k,:)';
        K_k = (P*psi_k)/(psi_k'*P*psi_k+lambda);
        teta_v(:,k)=teta_v(:,k-1)+K_k*(input.tau6(k)-psi_k'*teta_v(:,k-1));
        P=(P-(P*(psi_k*psi_k')*P)/(psi_k'*P*psi_k+lambda))/lambda;
    end
    
    theta6 = teta_v(:,end);
    
    etau1 = psi1(0.7*l+1:end,:)*theta1;
    etau2 = psi2(0.7*l+1:end,:)*theta2;
    etau3 = psi3(0.7*l+1:end,:)*theta3;
    etau4 = psi4(0.7*l+1:end,:)*theta4;
    etau5 = psi5(0.7*l+1:end,:)*theta5;
    etau6 = psi6(0.7*l+1:end,:)*theta6;
    
    %Plotting
    plot(input.tout(0.7*l+1:end,:),etau1);
    hold on
    plot(input.tout(0.7*l+1:end,:),input.tau1(0.7*l+1:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)') 
    figure
    plot(input.tout(0.7*l+1:end,:),etau2);
    hold on
    plot(input.tout(0.7*l+1:end,:),input.tau2(0.7*l+1:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l+1:end,:),etau3);
    hold on
    plot(input.tout(0.7*l+1:end,:),input.tau3(0.7*l+1:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l+1:end,:),etau4);
    hold on
    plot(input.tout(0.7*l+1:end,:),input.tau4(0.7*l+1:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l+1:end,:),etau5);
    hold on
    plot(input.tout(0.7*l+1:end,:),input.tau5(0.7*l+1:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    figure
    plot(input.tout(0.7*l+1:end,:),etau6);
    hold on
    plot(input.tout(0.7*l+1:end,:),input.tau6(0.7*l+1:end,:));
    grid on
    legend({'Estimated Torque','Measured Torque'})
    xlabel('Time (s)') 
    ylabel('Torque (N.m)')
    
    %Curve Fitness
    FIT = fitness([etau1,etau2,etau3,etau4,etau5,etau6],[input.tau1(0.7*l+1:end,:),input.tau2(0.7*l+1:end,:)...
        ,input.tau3(0.7*l+1:end,:),input.tau4(0.7*l+1:end,:),input.tau5(0.7*l+1:end,:),input.tau6(0.7*l+1:end,:)])
    
end