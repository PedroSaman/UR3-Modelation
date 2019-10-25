function [q,d_q,d_d_q,torque,time,omega,d_omega,alpha] = Data_Preparation()
    % This function calculates all needed position, speed, aceleration, and
    % mesured torque to be used in Dynamic_Model.m file. At the end of this
    % file, Pre_Calculations.m is called to pre calculate some of the
    % needed data used in Dynamic_Model.m function, these data is part of
    % the Newton-Euler formulation.
    
    % The format required here is: a single matrix called data located in 
    % /Documents/Git/UR3-Modelation/MAT files with n lines and 19 columns
    % the first one is the time stamp after that 3 by 3: angle, speed and
    % mesured torque for each one of the 6 joints.
    
    %data_aleatorios.mat
    %data_base.mat
    %data_j5.mat
    %id_data
    %data_toolbox.mat
    load ('~/Documents/Git/UR3-Modelation/MAT files/data_toolbox.mat');
    
    time = data(:,1); %#ok<NODEF,*NASGU>
    theta_J1 = data(:,2);
    d_theta_J1 = data(:,3);
    torque_J1 = data(:,4);
    theta_J2 = data(:,5);
    d_theta_J2 = data(:,6);
    torque_J2 = data(:,7);
    theta_J3 = data(:,8);
    d_theta_J3 = data(:,9);
    torque_J3 = data(:,10);
    theta_J4 = data(:,11);
    d_theta_J4 = data(:,12);
    torque_J4 = data(:,13);
    theta_J5 = data(:,14);
    d_theta_J5 = data(:,15);
    torque_J5 = data(:,16);
    theta_J6 = data(:,17);
    d_theta_J6 = data(:,18);
    torque_J6 = data(:,19);

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

    q = [theta_J1 theta_J2 theta_J3 theta_J4 theta_J5 theta_J6];
    d_q = [d_theta_J1 d_theta_J2 d_theta_J3 d_theta_J4 d_theta_J5 d_theta_J6];
    d_d_q = [d_dtheta_J1 d_dtheta_J2 d_dtheta_J3 d_dtheta_J4 d_dtheta_J5 d_dtheta_J6];
    torque = [torque_J1 torque_J2 torque_J3 torque_J4 torque_J5 torque_J6]; 
    
    %Pre_Calculations
    [omega,d_omega,alpha] = Pre_Calculations(q,d_q,d_d_q,dt);
end