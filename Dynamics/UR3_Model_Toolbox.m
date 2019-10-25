function [theta,q,d_q,d_d_q,toolbox_torque] = UR3_Model_Toolbox(q,d_q,d_d_q)
    % Using the Denavith Hatenberg Matrix defined in the path below, this
    % function create the dynamic model for the robot, write the torque
    % for each joint given the position, speed and aceleration disired q,
    % d_q and d_d_q. Here is possible to change gravity, center of mass,
    % mass, inertia matrix for validation purposes.
    % theta: dynamic parameters necessary to the Dynamic_Model.m function  
    
    load '~/Documents/Git/UR3-Modelation/MAT files/Denavith_Hatenberg_Matrix.mat';
    
    q(:,1) = 0;
    q(:,2) = 0;
    q(:,3) = 0;
    q(:,4) = 0;
    q(:,5) = 0;
    q(:,6) = 0;
    
    d_q(:,1) = 0;
    d_q(:,2) = 0;
    d_q(:,3) = 0;
    d_q(:,4) = 0;
    d_q(:,5) = 0;
    d_q(:,6) = 0;
    
    d_d_q(:,1) = 0;
    d_d_q(:,2) = 0;
    d_d_q(:,3) = 0;
    d_d_q(:,4) = 0;
    d_d_q(:,5) = 0;
    d_d_q(:,6) = 0;
   
    theta = zeros(1,96);
    robot = robotics.RigidBodyTree("DataFormat","row");
    robot.Gravity = [0 0 -9.81];

    body1 = robotics.RigidBody('body1');
    jnt1 = robotics.Joint('jnt1','revolute');
    body1.Mass = 1;
    body1.CenterOfMass = [0 0 0];
    body1.Inertia = [1 1 1 0 0 0];

    body2 = robotics.RigidBody('body2');
    jnt2 = robotics.Joint('jnt2','revolute');
    body2.Mass = 2;
    body2.CenterOfMass = [0 0 0];
    body2.Inertia = [1 1 1 0 0 0];

    body3 = robotics.RigidBody('body3');
    jnt3 = robotics.Joint('jnt3','revolute');
    body3.Mass = 3;
    body3.CenterOfMass = [0 0 0];
    body3.Inertia = [1 1 1 0 0 0];

    body4 = robotics.RigidBody('body4');
    jnt4 = robotics.Joint('jnt4','revolute');
    body4.Mass = 4;
    body4.CenterOfMass = [0 0 0];
    body4.Inertia = [1 1 1 0 0 0];

    body5 = robotics.RigidBody('body5');
    jnt5 = robotics.Joint('jnt5','revolute');
    body5.Mass = 5;
    body5.CenterOfMass = [0 0 0];
    body5.Inertia = [1 1 1 0 0 0];

    body6 = robotics.RigidBody('body6');
    jnt6 = robotics.Joint('jnt6','revolute');
    body6.Mass = 6;
    body6.CenterOfMass = [0 0 0];
    body6.Inertia = [1 1 1 0 0 0];

    body7 = robotics.RigidBody('body7');
    jnt7 = robotics.Joint('jnt7','revolute');
    body7.Mass = 0;
    body7.CenterOfMass = [0 0 0];
    body7.Inertia = [0 0 0 0 0 0];

    setFixedTransform(jnt1,dh(1,:),'dh');
    setFixedTransform(jnt2,dh(2,:),'dh');
    setFixedTransform(jnt3,dh(3,:),'dh');
    setFixedTransform(jnt4,dh(4,:),'dh');
    setFixedTransform(jnt5,dh(5,:),'dh');
    setFixedTransform(jnt6,dh(6,:),'dh');
    setFixedTransform(jnt7,dh(7,:),'dh');

    jnt3.HomePosition = -pi/2;
    jnt5.HomePosition = -pi/2;
    body1.Joint = jnt1;
    body2.Joint = jnt2;
    body3.Joint = jnt3;
    body4.Joint = jnt4;
    body5.Joint = jnt5;
    body6.Joint = jnt6;
    body7.Joint = jnt7;

    addBody(robot,body1,'base')
    addBody(robot,body2,'body1')
    addBody(robot,body3,'body2')
    addBody(robot,body4,'body3')
    addBody(robot,body5,'body4')
    addBody(robot,body6,'body5')
    addBody(robot,body7,'body6')

    toolbox_torque = inverseDynamics(robot,[0,q(1,1),q(1,2),q(1,3),q(1,4),q(1,5),q(1,6)] ...
    ,[d_q(1,1),d_q(1,2),d_q(1,3),d_q(1,4),d_q(1,5),d_q(1,6),0] ...
    ,[d_d_q(1,1),d_d_q(1,2),d_d_q(1,3),d_d_q(1,4),d_d_q(1,5),d_d_q(1,6),0]);
    show(robot,[0,q(1,1),q(1,2),q(1,3),q(1,4),q(1,5),q(1,6)]);
    
    toolbox_torque(1:end-1)
    

    theta(1,1:18) = [body1.CenterOfMass body2.CenterOfMass body3.CenterOfMass body4.CenterOfMass body5.CenterOfMass body6.CenterOfMass];
    theta(1,19:36) = [0 0 0.152 0.244 0 0 0.213 0 0 0 0 0.112 0 0 0.085 0 0 0.082];
    theta(1,37:45) = [body1.Inertia(1) body1.Inertia(6) body1.Inertia(5) body1.Inertia(6) body1.Inertia(2) body1.Inertia(4) body1.Inertia(5) body1.Inertia(4) body1.Inertia(3)];
    theta(1,46:54) = [body2.Inertia(1) body2.Inertia(6) body2.Inertia(5) body2.Inertia(6) body2.Inertia(2) body2.Inertia(4) body2.Inertia(5) body2.Inertia(4) body2.Inertia(3)];
    theta(1,55:63) = [body3.Inertia(1) body3.Inertia(6) body3.Inertia(5) body3.Inertia(6) body3.Inertia(2) body3.Inertia(4) body3.Inertia(5) body3.Inertia(4) body3.Inertia(3)];
    theta(1,64:72) = [body4.Inertia(1) body4.Inertia(6) body4.Inertia(5) body4.Inertia(6) body4.Inertia(2) body4.Inertia(4) body4.Inertia(5) body4.Inertia(4) body4.Inertia(3)];
    theta(1,73:81) = [body5.Inertia(1) body5.Inertia(6) body5.Inertia(5) body5.Inertia(6) body5.Inertia(2) body5.Inertia(4) body5.Inertia(5) body5.Inertia(4) body5.Inertia(3)];
    theta(1,82:90) = [body6.Inertia(1) body6.Inertia(6) body6.Inertia(5) body6.Inertia(6) body6.Inertia(2) body6.Inertia(4) body6.Inertia(5) body6.Inertia(4) body6.Inertia(3)];
    theta(1,91:96) = [body1.Mass body2.Mass body3.Mass body4.Mass body5.Mass body6.Mass];

end