function [Tau] = Fitness_Function(theta,q,omega,d_omega,alpha,f7,t7,m_torque)
                                    
    torque = Dynamic_Model(theta,q,omega,d_omega,alpha,f7,t7);
    Tau = zeros(1,6);
    for i=1:length(q)-1
        for j=1:6
            Tau(j) = Tau(j) + (torque(i,j) - m_torque(i,j)).^2;
        end
    end
    
    Tau/length(q)
    Tau = sum(Tau)/length(q)/6;
end