function [R] = Direct_Kinematics(theta)
%Given 6 Theta angles (deg), this function will return the position (meters) 
%and orientation (deg) coordinates as a vector R = %[X,Y,Z,Rx,Ry,Rz]
dh = [0,0.1520,0,-pi/2;0,0,0.2440,0;0,0,0.2130,0;0,0.1120,0,-pi/2;0,0.0850,0,pi/2;0,0.0820,0,0];

%Trasportation Matrix
T1 = [cos(theta(1)), -sin(theta(1)), 0, 0; sin(theta(1)), cos(theta(1)), 0, 0; 0, 0, 1, dh(1,2); 0, 0, 0, 1];
T2 = [cos(theta(2)), -sin(theta(2)), 0, dh(2-1,3); sin(theta(2))*cos(dh(2-1,4)), cos(theta(2))*cos(dh(2-1,4)), -sin(dh(2-1,4)), -sin(dh(2-1,4))*dh(2,2); sin(theta(2))*sin(dh(2-1,4)), cos(theta(2))*sin(dh(2-1,4)), cos(dh(2-1,4)), cos(dh(2-1,4))*dh(2,2); 0, 0, 0, 1];
T3 = [cos(theta(3)), -sin(theta(3)), 0, dh(3-1,3); sin(theta(3))*cos(dh(3-1,4)), cos(theta(3))*cos(dh(3-1,4)), -sin(dh(3-1,4)), -sin(dh(3-1,4))*dh(3,2); sin(theta(3))*sin(dh(3-1,4)), cos(theta(3))*sin(dh(3-1,4)), cos(dh(3-1,4)), cos(dh(3-1,4))*dh(3,2); 0, 0, 0, 1];
T4 = [cos(theta(4)), -sin(theta(4)), 0, dh(4-1,3); sin(theta(4))*cos(dh(4-1,4)), cos(theta(4))*cos(dh(4-1,4)), -sin(dh(4-1,4)), -sin(dh(4-1,4))*dh(4,2); sin(theta(4))*sin(dh(4-1,4)), cos(theta(4))*sin(dh(4-1,4)), cos(dh(4-1,4)), cos(dh(4-1,4))*dh(4,2); 0, 0, 0, 1];
T5 = [cos(theta(5)), -sin(theta(5)), 0, dh(5-1,3); sin(theta(5))*cos(dh(5-1,4)), cos(theta(5))*cos(dh(5-1,4)), -sin(dh(5-1,4)), -sin(dh(5-1,4))*dh(5,2); sin(theta(5))*sin(dh(5-1,4)), cos(theta(5))*sin(dh(5-1,4)), cos(dh(5-1,4)), cos(dh(5-1,4))*dh(5,2); 0, 0, 0, 1];
T6 = [cos(theta(6)), -sin(theta(6)), 0, dh(6-1,3); sin(theta(6))*cos(dh(6-1,4)), cos(theta(6))*cos(dh(6-1,4)), -sin(dh(6-1,4)), -sin(dh(6-1,4))*dh(6,2); sin(theta(6))*sin(dh(6-1,4)), cos(theta(6))*sin(dh(6-1,4)), cos(dh(6-1,4)), cos(dh(6-1,4))*dh(6,2); 0, 0, 0, 1];

%Correting Y and X axis
% T = [-1 0 0 0;0 -1 0 0; 0 0 1 0; 0 0 0 1];

%Final homogeneous matrix
Transf = T1*T2*T3*T4*T5*T6;

%Cartesian Coordinates
X = Transf(1,4);
Y = Transf(2,4);
Z = Transf(3,4);

%Orientation Coordinates
r = tr2rpy(Transf,'arm');
%Output
R = [X,Y,Z,r(1),r(2),r(3)];
end