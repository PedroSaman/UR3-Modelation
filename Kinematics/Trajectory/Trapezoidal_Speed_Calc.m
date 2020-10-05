function [q,t] = Trapezoidal_Speed_Calc(qi,qf,ac,tf,dt)
    % This functions performs the necessary calculations to the trajectory
    % with the parameters defined in Trajectory_Generation.m file

    tc = tf/2 - sqrt((ac*tf^2-4*(qf-qi))/ac)/2;
    t1 = (0:dt:tc);
    t2 = (t1(end)+dt:dt:(tf-tc));
    t3 = (t2(end)+dt:dt:tf);
    q1 = zeros(length(t1),1);
    q2 = zeros(length(t2),1);
    q3 = zeros(length(t3),1);

    for i=1:length(t1)
        q1(i) = qi+ac*t1(i)^2/2;
    end

    for i=1:length(t2)
        q2(i) = qi +ac*tc*(t2(i) - tc/2);
    end

    for i=1:length(t3)
        q3(i) = qf - ac*(t3(i)-tf)^2/2;
    end

    q = [q1;q2;q3];
    t = [t1,t2,t3];
end