clear
clc

addpath '~/Documents/Git/UR3-Modelation/Dynamics';

ObjectiveFunction = @(theta) FitnessFunction_SA(theta);
options = optimoptions(@simulannealbnd, ...
                     'PlotFcn',{@saplotbestf,@saplottemperature,@saplotf,@saplotstopping}, ...
                     'InitialTemperature', 10, 'TemperatureFcn', {@temperaturefast});
                 
load('Results/resultado138.mat')
x0 = theta;

tic
[theta,fval,exitFlag,output] = simulannealbnd(ObjectiveFunction,x0,[],[],options);
toc