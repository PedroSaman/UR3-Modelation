function [traj] = Trajectory_Generation()
    % This function defines the position, speed and aceleration functions
    % in time so that the speed have trapezoidal shape. Variables needed to
    % be defined: 
    %   - Initial[xi,yi,zi] and Final[xf,yf,zf] points to X,Y,Z of the end effector tool;
    %   - Time to perform[tf] the trajectory and sampling time[dt];
    %   - Aceleration[ac], If the initial point < final point ac must be negative;

    xi = -0.112;
    xf = 0.112;
    yi = 0.405;
    yf = 0.405;
    zi = 0.215;
    zf = 0.215;
    rxi = -pi/2;
    rxf = -pi/2;
    ryi = 0;
    ryf = 0;
    rzi = -pi/2;
    rzf = -pi/2;
    acx = 0.1;
    acy = 0.1;
    acz = 0.1;
    tf = 5;
    dt = 0.05;
    [traj.x,~] = Trapezoidal_Speed_Calc(xi,xf,acx,tf,dt);
    [traj.y,~] = Trapezoidal_Speed_Calc(yi,yf,acy,tf,dt);
    [traj.z,~] = Trapezoidal_Speed_Calc(zi,zf,acz,tf,dt);
    [traj.rx,~] = Trapezoidal_Speed_Calc(rxi,rxf,acx,tf,dt);
    [traj.ry,~] = Trapezoidal_Speed_Calc(ryi,ryf,acy,tf,dt);
    [traj.rz,t] = Trapezoidal_Speed_Calc(rzi,rzf,acz,tf,dt);
    
    traj.t = t';
    
%     plot(tx,x);
%     figure();
%     plot(ty,y);
%     figure();
%     plot(tz,z);
    traj.dx = diff(traj.x);
    traj.dx(end+1) = 0;
    traj.dy = diff(traj.y);
    traj.dy(end+1) = 0;
    traj.dz = diff(traj.z);
    traj.dz(end+1) = 0;
    traj.ddx = diff(traj.dx);
    traj.ddx(end+1) = 0;
    traj.ddy = diff(traj.dy);
    traj.ddy(end+1) = 0;
    traj.ddz = diff(traj.dz);
    traj.ddz(end+1) = 0;
    
    traj.drx = diff(traj.rx);
    traj.drx(end+1) = 0;
    traj.dry = diff(traj.ry);
    traj.dry(end+1) = 0;
    traj.drz = diff(traj.rz);
    traj.drz(end+1) = 0;
    traj.ddrx = diff(traj.drx);
    traj.ddrx(end+1) = traj.ddrx(end);
    traj.ddry = diff(traj.dry);
    traj.ddry(end+1) = traj.ddry(end);
    traj.ddrz = diff(traj.drz);
    traj.ddrz(end+1) = traj.ddrz(end);
    
    traj.rx = traj.rx;
    traj.ry = traj.ry;
    traj.rz = traj.rz;
end