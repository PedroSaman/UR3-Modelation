function [omega,d_omega,alpha] = Pre_Calculations(q,d_q,d_d_q,dt)
    % This function pre calculate omega, d_omega and alpha to be used in
    % Dymanic_Model.m function. It needs q, d_q, d_d_q and time vectors
    
    %z0 axis orientation
    z0 = [0;0;1];

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
        [R0_1,R1_2,R2_3,R3_4,R4_5,~] = Rotation_Matrices(q(i,:));

        omega1(1:3,i) = z0*d_q(i,1);
        omega2(1:3,i) = (R0_1'*omega1(1:3,i) + z0*d_q(i,2));
        omega3(1:3,i) = (R1_2'*omega2(1:3,i) + z0*d_q(i,3));
        omega4(1:3,i) = (R2_3'*omega3(1:3,i) + z0*d_q(i,4));
        omega5(1:3,i) = (R3_4'*omega4(1:3,i) + z0*d_q(i,5));
        omega6(1:3,i) = (R4_5'*omega5(1:3,i) + z0*d_q(i,6));
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
        [R0_1,R1_2,R2_3,R3_4,R4_5,~] = Rotation_Matrices(q(i,:));
        
        alpha1(1:3,i) = z0*d_d_q(i,1) + cross(omega1(1:3,i),z0*d_q(i,1));
        alpha2(1:3,i) = (R0_1'*alpha1(1:3,i) + z0*d_d_q(i,2) + cross(omega2(1:3,i),z0*d_q(i,2)));
        alpha3(1:3,i) = (R1_2'*alpha2(1:3,i) + z0*d_d_q(i,3) + cross(omega3(1:3,i),z0*d_q(i,3)));
        alpha4(1:3,i) = (R2_3'*alpha3(1:3,i) + z0*d_d_q(i,4) + cross(omega4(1:3,i),z0*d_q(i,4)));
        alpha5(1:3,i) = (R3_4'*alpha4(1:3,i) + z0*d_d_q(i,5) + cross(omega5(1:3,i),z0*d_q(i,5)));
        alpha6(1:3,i) = (R4_5'*alpha5(1:3,i) + z0*d_d_q(i,6) + cross(omega6(1:3,i),z0*d_q(i,6)));
    end

    %Function output
    d_omega = [d_omega1;d_omega2;d_omega3;d_omega4;d_omega5;d_omega6];
    omega = [omega1;omega2;omega3;omega4;omega5;omega6];
    alpha = [alpha1;alpha2;alpha3;alpha4;alpha5;alpha6];
end