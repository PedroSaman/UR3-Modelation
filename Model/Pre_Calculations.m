function [omega,d_omega,alpha] = Pre_Calculations(q,d_q,d_d_q,dt)
    %This function pre calculate omega, d_omega and alpha to be used in
    %Function_Model function. It needs q, d_q, d_d_q and time vectors
   
    %z0 axis orientation
    z0 = [0;0;1];
        
    %Rotation Matrix
    R0_1 = ([cos(q(1)-pi/2) -sin(q(1)-pi/2) 0;sin(q(1)-pi/2) cos(q(1)-pi/2) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R1_2 = ([cos(q(2)+pi/2) -sin(q(2)+pi/2) 0;sin(q(2)+pi/2) cos(q(2)+pi/2) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R2_3 = ([cos(q(3)) -sin(q(3)) 0;sin(q(3)) cos(q(3)) 0; 0 0 1]*[1 0 0; 0 cos(0) -sin(0); 0 sin(0) cos(0)]);
    R3_4 = ([cos(q(4)+pi/2) -sin(q(4)+pi/2) 0;sin(q(4)+pi/2) cos(q(4)+pi/2) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R4_5 = ([cos(q(5)+pi) -sin(q(5)+pi) 0;sin(q(5)+pi) cos(q(5)+pi) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R5_6 = ([cos(q(6)) -sin(q(6)) 0;sin(q(6)) cos(q(6)) 0; 0 0 1]*[1 0 0; 0 cos(pi/2) -sin(pi/2); 0 sin(pi/2) cos(pi/2)]);
    R0_2 = R0_1*R1_2;
    R0_3 = R0_2*R2_3;
    R0_4 = R0_3*R3_4;
    R0_5 = R0_4*R4_5;
    R0_6 = R0_5*R5_6;
    
    %B parameter
    b1 = R0_1'*z0;     
    b2 = R0_2'*R0_1*z0;
    b3 = R0_3'*R0_2*z0;
    b4 = R0_4'*R0_3*z0;
    b5 = R0_5'*R0_4*z0;
    b6 = R0_6'*R0_5*z0;
    
    %Pre alocation
    omega1 = zeros([3,length(q)]);
    omega2 = zeros([3,length(q)]);
    omega3 = zeros([3,length(q)]);
    omega4 = zeros([3,length(q)]);
    omega5 = zeros([3,length(q)]);
    omega6 = zeros([3,length(q)]);
    alpha1 = zeros([3,length(q)]);
    alpha2 = zeros([3,length(q)]);
    alpha3 = zeros([3,length(q)]);
    alpha4 = zeros([3,length(q)]);
    alpha5 = zeros([3,length(q)]);
    alpha6 = zeros([3,length(q)]);
    
    %omega calculation
    for i=1:length(q)-1
        omega1(1:3,i) = b1*d_q(i,1);
        omega2(1:3,i) = R1_2'*omega1(1:3,i) + b2*d_q(i,2);
        omega3(1:3,i) = R2_3'*omega2(1:3,i) + b3*d_q(i,3);
        omega4(1:3,i) = R3_4'*omega3(1:3,i) + b4*d_q(i,4);
        omega5(1:3,i) = R4_5'*omega4(1:3,i) + b5*d_q(i,5);
        omega6(1:3,i) = R5_6'*omega5(1:3,i) + b6*d_q(i,6);
    end
    
    %d_omega calculation
    for i=1:length(q)-1
        d_omega1 = (diff(omega1.')/dt(i))';
        d_omega2 = (diff(omega2.')/dt(i))';
        d_omega3 = (diff(omega3.')/dt(i))';
        d_omega4 = (diff(omega4.')/dt(i))';
        d_omega5 = (diff(omega5.')/dt(i))';
        d_omega6 = (diff(omega6.')/dt(i))';
    end
    
    %Alpha calculation
    for i=1:length(q)-1
        alpha1(1:3,i) = b1*d_d_q(i,1) + cross(omega1(1:3,i),b1*d_q(i,1));
        alpha2(1:3,i) = R1_2'*alpha1(1:3,i) + b2*d_d_q(i,2) + cross(omega2(1:3,i),b2*d_q(i,2));
        alpha3(1:3,i) = R2_3'*alpha2(1:3,i) + b3*d_d_q(i,3) + cross(omega3(1:3,i),b3*d_q(i,3));
        alpha4(1:3,i) = R3_4'*alpha3(1:3,i) + b4*d_d_q(i,4) + cross(omega4(1:3,i),b4*d_q(i,4));
        alpha5(1:3,i) = R4_5'*alpha4(1:3,i) + b5*d_d_q(i,5) + cross(omega5(1:3,i),b5*d_q(i,5));
        alpha6(1:3,i) = R5_6'*alpha5(1:3,i) + b6*d_d_q(i,6) + cross(omega6(1:3,i),b6*d_q(i,6));
    end
    
    %Function output
    d_omega = [d_omega1;d_omega2;d_omega3;d_omega4;d_omega5;d_omega6];
    omega = [omega1;omega2;omega3;omega4;omega5;omega6];
    alpha = [alpha1;alpha2;alpha3;alpha4;alpha5;alpha6];
end