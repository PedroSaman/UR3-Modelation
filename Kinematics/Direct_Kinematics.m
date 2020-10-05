function [R] = Direct_Kinematics(theta)
%Given 6 Theta angles (deg), this function will return the position (meters) 
%and orientation (deg) coordinates as a vector R = %[X,Y,Z,Rx,Ry,Rz]
clc
load('DH_Toolbox.mat','dh');

theta = deg2rad(theta);

%Trasportation Matrix
T1 = [cos(theta(1,1)), -sin(theta(1,1)), 0, 0; sin(theta(1,1)), cos(theta(1,1)), 0, 0; 0, 0, 1, dh(1,2); 0, 0, 0, 1];
T2 = [cos(theta(1,2)), -sin(theta(1,2)), 0, dh(2-1,3); sin(theta(1,2))*cos(dh(2-1,4)), cos(theta(1,2))*cos(dh(2-1,4)), -sin(dh(2-1,4)), -sin(dh(2-1,4))*dh(2,2); sin(theta(1,2))*sin(dh(2-1,4)), cos(theta(1,2))*sin(dh(2-1,4)), cos(dh(2-1,4)), cos(dh(2-1,4))*dh(2,2); 0, 0, 0, 1];
T3 = [cos(theta(1,3)), -sin(theta(1,3)), 0, dh(3-1,3); sin(theta(1,3))*cos(dh(3-1,4)), cos(theta(1,3))*cos(dh(3-1,4)), -sin(dh(3-1,4)), -sin(dh(3-1,4))*dh(3,2); sin(theta(1,3))*sin(dh(3-1,4)), cos(theta(1,3))*sin(dh(3-1,4)), cos(dh(3-1,4)), cos(dh(3-1,4))*dh(3,2); 0, 0, 0, 1];
T4 = [cos(theta(1,4)), -sin(theta(1,4)), 0, dh(4-1,3); sin(theta(1,4))*cos(dh(4-1,4)), cos(theta(1,4))*cos(dh(4-1,4)), -sin(dh(4-1,4)), -sin(dh(4-1,4))*dh(4,2); sin(theta(1,4))*sin(dh(4-1,4)), cos(theta(1,4))*sin(dh(4-1,4)), cos(dh(4-1,4)), cos(dh(4-1,4))*dh(4,2); 0, 0, 0, 1];
T5 = [cos(theta(1,5)), -sin(theta(1,5)), 0, dh(5-1,3); sin(theta(1,5))*cos(dh(5-1,4)), cos(theta(1,5))*cos(dh(5-1,4)), -sin(dh(5-1,4)), -sin(dh(5-1,4))*dh(5,2); sin(theta(1,5))*sin(dh(5-1,4)), cos(theta(1,5))*sin(dh(5-1,4)), cos(dh(5-1,4)), cos(dh(5-1,4))*dh(5,2); 0, 0, 0, 1];
T6 = [cos(theta(1,6)), -sin(theta(1,6)), 0, dh(6-1,3); sin(theta(1,6))*cos(dh(6-1,4)), cos(theta(1,6))*cos(dh(6-1,4)), -sin(dh(6-1,4)), -sin(dh(6-1,4))*dh(6,2); sin(theta(1,6))*sin(dh(6-1,4)), cos(theta(1,6))*sin(dh(6-1,4)), cos(dh(6-1,4)), cos(dh(6-1,4))*dh(6,2); 0, 0, 0, 1];

%Correting Y and X axis
% T = [-1 0 0 0;0 -1 0 0; 0 0 1 0; 0 0 0 1];

%Final homogeneous matrix
Transf = T1*T2*T3*T4*T5*T6;



%Cartesian Coordinates
X = Transf(1,4);
Y = Transf(2,4);
Z = Transf(3,4);

%Orientation Coordinates
Rz = atan2(-1*Transf(3,1),sqrt(Transf(1,1).^2+Transf(2,1).^2));
if (Rz == pi)
    Ry = 0;
    Rx = atan2(Transf(1,2),Transf(2,2));
elseif (Rz == -pi)
    Ry = 0;
    Rx = -atan2(Transf(1,2),Transf(2,2));
else
    Ry = atan2(Transf(2,1)/cos(Rz),Transf(1,1)/cos(Rz));
    Rx = atan2(Transf(3,2)/cos(Rz),Transf(3,3)/cos(Rz));
end

%Output
R = [X,Y,Z,Rx,Ry,Rz];

end