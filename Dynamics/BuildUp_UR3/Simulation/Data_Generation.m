function [output] = Data_Generation()    
% Simulate a trajectory for 6DOF manipulator using a combination of sines
% and a poly. Works with the simulink file DOF6_Trajectory_simulation.slx

    pol = poly([0 1 2.5 5 10 15 20 25 30 35 40 45 50 55 57.5 59 60]);
    t = [0:0.05:60];
    pol = polyval(pol,t)/200000000000000000000;
    pol = [t' pol'];
    simout = sim('DOF6_Trajectory_simulation.slx','SrcWorkspace','current');
    clc
    
    tout = simout.tout;
    pos1 = simout.q1;
    pos2 = simout.q2;
    pos3 = simout.q3;
    pos4 = simout.q4;
    pos5 = simout.q5;
    pos6 = simout.q6;
    vel1 = simout.dq1;
    vel1(1)=vel1(2);
    vel2 = simout.dq2;
    vel2(1)=vel2(2);
    vel3 = simout.dq3;
    vel3(1)=vel3(2);
    vel4 = simout.dq4;
    vel4(1)=vel4(2);
    vel5 = simout.dq5;
    vel5(1)=vel5(2);
    vel6 = simout.dq6;
    vel6(1)=vel6(2);
    ace1 = simout.ddq1;
    ace1(1) = ace1(3);
    ace1(2) = ace1(3);
    ace2 = simout.ddq2;
    ace2(1) = ace2(3);
    ace2(2) = ace2(3);
    ace3 = simout.ddq3;
    ace3(1) = ace3(3);
    ace3(2) = ace3(3);
    ace4 = simout.ddq4;
    ace4(1) = ace4(3);
    ace4(2) = ace4(3);
    ace5 = simout.ddq5;
    ace5(1) = ace5(3);
    ace5(2) = ace5(3);
    ace6 = simout.ddq6;
    ace6(1) = ace6(3);
    ace6(2) = ace6(3);
    
    output.tout = tout;
    output.pos1 = pos1;
    output.pos2 = pos2;
    output.pos3 = pos3;
    output.pos4 = pos4;
    output.pos5 = pos5;
    output.pos6 = pos6;
    output.vel1 = vel1;
    output.vel2 = vel2;
    output.vel3 = vel3;
    output.vel4 = vel4;
    output.vel5 = vel5;
    output.vel6 = vel6;
    output.ace1 = ace1;
    output.ace2 = ace2;
    output.ace3 = ace3;
    output.ace4 = ace4;
    output.ace5 = ace5;
    output.ace6 = ace6;
end