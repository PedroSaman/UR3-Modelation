function [Tau] = FitnessFunction_SA(theta)
    
    alpha = 0;
    load('~/Documents/Git/UR3-Modelation/MAT files/training_data.mat');
    normalization = [square_mean(m_torque(:,1)) square_mean(m_torque(:,2)) square_mean(m_torque(:,3)) ...
    square_mean(m_torque(:,4)) square_mean(m_torque(:,5)) square_mean(m_torque(:,6))];
    addpath '~/Documents/Git/UR3-Modelation/Dynamics'
    torque = Dynamic_Model(theta,q,omega,d_omega,alpha,[0;0;0],[0;0;0]);
    Tau = zeros(1,6);
    for i=1:length(q)-1
        for j=1:6
            Tau(j) = Tau(j) + ((torque(i,j) - m_torque(i,j)).^2);
        end
     end
    Tau = [Tau(1)/normalization(1) Tau(2)/normalization(2) Tau(3)/normalization(3) ...
            Tau(4)/normalization(4) Tau(5)/normalization(5) Tau(6)/normalization(6)];
    Tau/length(torque)
    Tau = sum(Tau)/length(torque)/6;
end