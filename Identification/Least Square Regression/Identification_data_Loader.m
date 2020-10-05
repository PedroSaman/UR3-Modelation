function [dataset]=Identification_data_Loader()
% Load the mass of data and organize them to identification

    load('~/Documents/Git/UR3-Modelation/MAT files/Data1.mat')
    dataset.tout = time;
    dataset.pos1 = q(:,1);
    dataset.pos2 = q(:,2);
    dataset.pos3 = q(:,3);
    dataset.pos4 = q(:,4);
    dataset.pos5 = q(:,5);
    dataset.pos6 = q(:,6);
    dataset.vel1 = d_q(:,1);
    dataset.vel2 = d_q(:,2);
    dataset.vel3 = d_q(:,3);
    dataset.vel4 = d_q(:,4);
    dataset.vel5 = d_q(:,5);
    dataset.vel6 = d_q(:,6);
    dataset.ace1 = d_d_q(:,1);
    dataset.ace2 = d_d_q(:,2);
    dataset.ace3 = d_d_q(:,3);
    dataset.ace4 = d_d_q(:,4);
    dataset.ace5 = d_d_q(:,5);
    dataset.ace6 = d_d_q(:,6);
    dataset.ace6(end+1) = dataset.ace6(end);
    dataset.ace5(end+1) = dataset.ace5(end);
    dataset.ace4(end+1) = dataset.ace4(end);
    dataset.ace3(end+1) = dataset.ace3(end);
    dataset.ace2(end+1) = dataset.ace2(end);
    dataset.ace1(end+1) = dataset.ace1(end);
    dataset.tau1 = m_torque(:,1);
    dataset.tau2 = m_torque(:,2);
    dataset.tau3 = m_torque(:,3);
    dataset.tau4 = m_torque(:,4);
    dataset.tau5 = m_torque(:,5);
    dataset.tau6 = m_torque(:,6);
end