function [R0_1,R1_2,R2_3,R3_4,R4_5,R5_6] = Rotation_Matrices(q)
%   Given the current rotation around all joints, this calculate the 
%   rotation transformation for each consecutive joint. Must be defined for
%   each particular robot.
    
    R0_1 = ([cos(q(1)) -sin(q(1)) 0;sin(q(1)) cos(q(1)) 0; 0 0 1]*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R1_2 = ([cos(q(2)) -sin(q(2)) 0;sin(q(2)) cos(q(2)) 0; 0 0 1]*[1 0 0; 0 cos(0)     -sin(0)    ; 0 sin(0)     cos(0)    ]);
    R2_3 = ([cos(q(3)) -sin(q(3)) 0;sin(q(3)) cos(q(3)) 0; 0 0 1]*[1 0 0; 0 cos(0)     -sin(0)    ; 0 sin(0)     cos(0)    ]);
    R3_4 = ([cos(q(4)) -sin(q(4)) 0;sin(q(4)) cos(q(4)) 0; 0 0 1]*[1 0 0; 0 cos(-pi/2) -sin(-pi/2); 0 sin(-pi/2) cos(-pi/2)]);
    R4_5 = ([cos(q(5)) -sin(q(5)) 0;sin(q(5)) cos(q(5)) 0; 0 0 1]*[1 0 0; 0 cos(pi/2)  -sin(pi/2) ; 0 sin(pi/2)  cos(pi/2) ]);
    R5_6 = ([cos(q(6)) -sin(q(6)) 0;sin(q(6)) cos(q(6)) 0; 0 0 1]*[1 0 0; 0 cos(0)     -sin(0)    ; 0 sin(0)     cos(0)    ]);

end