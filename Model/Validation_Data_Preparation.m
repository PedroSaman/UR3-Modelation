function [q,d_q,d_d_q,torque,time,omega,d_omega,alpha] = Validation_Data_Preparation()
    clear
    clc
    %dados_aleatorios.mat
    %dados_base.mat
    %dados_j5.mat
    load ('~/Documents/Git/UR3-Modelation/MAT files/dados_base.mat');
    
    time = dados(:,1);
    theta_J1 = dados(:,2);
    d_theta_J1 = dados(:,3);
    torque_J1 = dados(:,4);
    theta_J2 = dados(:,5);
    d_theta_J2 = dados(:,6);
    torque_J2 = dados(:,7);
    theta_J3 = dados(:,8);
    d_theta_J3 = dados(:,9);
    torque_J3 = dados(:,10);
    theta_J4 = dados(:,11);
    d_theta_J4 = dados(:,12);
    torque_J4 = dados(:,13);
    theta_J5 = dados(:,14);
    d_theta_J5 = dados(:,15);
    torque_J5 = dados(:,16);
    theta_J6 = dados(:,17);
    d_theta_J6 = dados(:,18);
    torque_J6 = dados(:,19);

    dt = diff(time);
    d_dtheta_J1 = diff(d_theta_J1);
    d_dtheta_J2 = diff(d_theta_J2);
    d_dtheta_J3 = diff(d_theta_J3);
    d_dtheta_J4 = diff(d_theta_J4);
    d_dtheta_J5 = diff(d_theta_J5);
    d_dtheta_J6 = diff(d_theta_J6);

    for i = 1:length(time) -1 
        d_dtheta_J1(i) = d_dtheta_J1(i)/dt(i);
        d_dtheta_J2(i) = d_dtheta_J2(i)/dt(i);
        d_dtheta_J3(i) = d_dtheta_J3(i)/dt(i);
        d_dtheta_J4(i) = d_dtheta_J4(i)/dt(i);
        d_dtheta_J5(i) = d_dtheta_J5(i)/dt(i);
        d_dtheta_J6(i) = d_dtheta_J6(i)/dt(i);
    end

    q = [theta_J1 theta_J2+pi/2 theta_J3 theta_J4+pi/2 theta_J5 theta_J6];
    d_q = [d_theta_J1 d_theta_J2 d_theta_J3 d_theta_J4 d_theta_J5 d_theta_J6];
    d_d_q = [d_dtheta_J1 d_dtheta_J2 d_dtheta_J3 d_dtheta_J4 d_dtheta_J5 d_dtheta_J6];
    torque = [torque_J1 torque_J2 torque_J3 torque_J4 torque_J5 torque_J6]; 
    
    %Pre_Calculations
    [omega,d_omega,alpha] = Pre_Calculations(q,d_q,d_d_q,dt);
end