function [theta1,theta2,theta3,theta4,theta5,theta6] = Identification6DOF()
    
    clc
    clear
    load('dataDOF6.mat');
    [input] = data_preparation();
    [psi1,psi2,psi3,psi4,psi5,psi6] = PsiMatrix(input);
    
    theta1 = pinv(psi1)*tau1;
    theta2 = pinv(psi2)*tau2;
    theta3 = pinv(psi3)*tau3;
    theta4 = pinv(psi4)*tau4;
    theta5 = pinv(psi5)*tau5;
    theta6 = pinv(psi6)*tau6;
    
    etau1 = psi1*theta1;
    etau2 = psi2*theta2;
    etau3 = psi3*theta3;
    etau4 = psi4*theta4;
    etau5 = psi5*theta5;
    etau6 = psi6*theta6;
    
end