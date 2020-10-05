function [J] = JacobianUR3(t)
%Calculate the Jacobian for UR3 in the joint configuration 't'.

dh = [0,0.1520,0,-pi/2;0,0,0.2440,0;0,0,0.2130,0;0,0.1120,0,-pi/2;0,0.0850,0,pi/2;0,0.0820,0,0];

%Trasportation Matrix
A1 = [cos(t(1)),-sin(t(1))*round(cos(dh(1,4))),sin(t(1))*sin(dh(1,4)),dh(1,3)*cos(t(1));sin(t(1)),cos(t(1))*round(cos(dh(1,4))),-cos(t(1))*sin(dh(1,4)),dh(1,3)*sin(t(1));0,sin(dh(1,4)),round(cos(dh(1,4))),dh(1,2);0,0,0,1];
A2 = [cos(t(2)),-sin(t(2))*round(cos(dh(2,4))),sin(t(2))*sin(dh(2,4)),dh(2,3)*cos(t(2));sin(t(2)),cos(t(2))*round(cos(dh(2,4))),-cos(t(2))*sin(dh(2,4)),dh(2,3)*sin(t(2));0,sin(dh(2,4)),round(cos(dh(2,4))),dh(2,2);0,0,0,1];
A3 = [cos(t(3)),-sin(t(3))*round(cos(dh(3,4))),sin(t(3))*sin(dh(3,4)),dh(3,3)*cos(t(3));sin(t(3)),cos(t(3))*round(cos(dh(3,4))),-cos(t(3))*sin(dh(3,4)),dh(3,3)*sin(t(3));0,sin(dh(3,4)),round(cos(dh(3,4))),dh(3,2);0,0,0,1];
A4 = [cos(t(4)),-sin(t(4))*round(cos(dh(4,4))),sin(t(4))*sin(dh(4,4)),dh(4,3)*cos(t(4));sin(t(4)),cos(t(4))*round(cos(dh(4,4))),-cos(t(4))*sin(dh(4,4)),dh(4,3)*sin(t(4));0,sin(dh(4,4)),round(cos(dh(4,4))),dh(4,2);0,0,0,1];
A5 = [cos(t(5)),-sin(t(5))*round(cos(dh(5,4))),sin(t(5))*sin(dh(5,4)),dh(5,3)*cos(t(5));sin(t(5)),cos(t(5))*round(cos(dh(5,4))),-cos(t(5))*sin(dh(5,4)),dh(5,3)*sin(t(5));0,sin(dh(5,4)),round(cos(dh(5,4))),dh(5,2);0,0,0,1];
A6 = [cos(t(6)),-sin(t(6))*round(cos(dh(6,4))),sin(t(6))*sin(dh(6,4)),dh(6,3)*cos(t(6));sin(t(6)),cos(t(6))*round(cos(dh(6,4))),-cos(t(6))*sin(dh(6,4)),dh(6,3)*sin(t(6));0,sin(dh(6,4)),round(cos(dh(6,4))),dh(6,2);0,0,0,1];

T1 = A1;
T2 = T1*A2;
T3 = T2*A3;
T4 = T3*A4;
T5 = T4*A5;
T6 = T5*A6;

z0 = [0;0;1];
z1 = T1(1:3,3);
z2 = T2(1:3,3);
z3 = T3(1:3,3);
z4 = T4(1:3,3);
z5 = T5(1:3,3);

p0 = [0;0;0];
p1 = T1(1:3,4);
p2 = T2(1:3,4);
p3 = T3(1:3,4);
p4 = T4(1:3,4);
p5 = T5(1:3,4);
P = T6(1:3,4);

J = double([cross(z0,P-p0),cross(z1,P-p1),cross(z2,P-p2),cross(z3,P-p3),cross(z4,P-p4),cross(z5,P-p5);z0,z1,z2,z3,z4,z5]);
end