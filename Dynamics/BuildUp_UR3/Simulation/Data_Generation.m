function [input] = Data_Generation()
% Simulate a trajectory for 6DOF manipulator using a combination of sines
% and a poly. Works with the simulink file DOF6_Trajectory_simulation.slx

    pol = poly([0 1 2.5 5 10 15 20 25 30 35 40 45 50 55 57.5 59 60]);
    t = [0:0.05:60];
    pol = polyval(pol,t)/200000000000000000000;
    pol = [t' pol'];
    pos_var = 0;
    spe_var = 0;
    ace_var = 0;
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
    
    input.tout = tout;
    input.pos1 = pos1;
    input.pos2 = pos2;
    input.pos3 = pos3;
    input.pos4 = pos4;
    input.pos5 = pos5;
    input.pos6 = pos6;
    input.vel1 = vel1;
    input.vel2 = vel2;
    input.vel3 = vel3;
    input.vel4 = vel4;
    input.vel5 = vel5;
    input.vel6 = vel6;
    input.ace1 = ace1;
    input.ace2 = ace2;
    input.ace3 = ace3;
    input.ace4 = ace4;
    input.ace5 = ace5;
    input.ace6 = ace6;
end