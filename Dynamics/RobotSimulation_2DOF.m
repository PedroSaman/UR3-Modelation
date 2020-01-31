function [t1,t2] = RobotSimulation_2DOF(q1,q2,dq1,dq2,ddq1,ddq2)
% Given the 2 DOF robot defined in this function and a trajectory
% this function gives the two output torques  
    
    % Robot and physics constants
    g = 10;
    lc1 = 1.5;
    lc2 = 2;
    l1 = 2;
    I1 = 0.2;
    I2 = 0.1;
    m1 = 0.5;
    m2 = 0.3;
    b1 = 0.05;
    b2 = 0.05;    
    t1 = zeros(length(q1),1);
    t2 = zeros(length(q1),1);
    
    for i=1:length(q1)
        theta1 = q1(i);
        theta2 = q2(i);
        omega1 = dq1(i);
        omega2 = dq2(i);
        alpha1 = ddq1(i);
        alpha2 = ddq2(i);      
        
        phi2 = [cos(theta2)*alpha1 alpha2 sin(theta2)*omega1^2 cos(theta1+theta2) omega2];
        phi1 = [cos(theta2)*alpha2 -sin(theta2)*omega2^2 cos(theta1) alpha1 omega1];
        reg2 = [l1*lc2*m2 lc2^2*m2+I2 l1*lc2*m2 g*lc2*m2 b2]';
        reg1 = [l1*lc2*m2 l1*lc2*m2 g*(l1*m2+lc1*m1) l1^2*m2+lc1^2*m1+I1 b1]';
        t2(i) = phi2*reg2;
        t1(i) = t2(i) + phi1*reg1;
    end
end

