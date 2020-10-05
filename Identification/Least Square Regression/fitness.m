function [FIT] = fitness(torque,m_torque)
    FIT1 = 100* (1-norm(torque(:,1)-m_torque(:,1))/norm(torque(:,1)-mean(torque(:,1))));
    FIT2 = 100* (1-norm(torque(:,2)-m_torque(:,2))/norm(torque(:,2)-mean(torque(:,2))));
    FIT3 = 100* (1-norm(torque(:,3)-m_torque(:,3))/norm(torque(:,3)-mean(torque(:,3))));
    FIT4 = 100* (1-norm(torque(:,4)-m_torque(:,4))/norm(torque(:,4)-mean(torque(:,4))));
    FIT5 = 100* (1-norm(torque(:,5)-m_torque(:,5))/norm(torque(:,5)-mean(torque(:,5))));
    FIT6 = 100* (1-norm(torque(:,6)-m_torque(:,6))/norm(torque(:,6)-mean(torque(:,6))));
    FIT = [FIT1,FIT2,FIT3,FIT4,FIT5,FIT6];
end