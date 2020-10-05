function [x,y,z,dx,dy,dz,ddx,ddy,ddz,tx,ty,tz,dt] = Trajectory_Generation()
    % This function defines the position, speed and aceleration functions
    % in time so that the speed have trapezoidal shape. Variables needed to
    % be defined: 
    %   - Initial[xi,yi,zi] and Final[xf,yf,zf] points to X,Y,Z of the end effector tool;
    %   - Time to perform[tf] the trajectory and sampling time[dt];
    %   - Aceleration[ac], If the initial point < final point ac must be negative;

    xi = 0.329;
    xf = -0.112;
    yi = 0.112;
    yf = 0.405;
    zi = 0.283;
    zf = 0.215;
    acx = -0.1;
    acy = 0.1;
    acz = -0.1;
    tf = 5;
    dt = 0.05;
    [x,tx] = Trapezoidal_Speed_Calc(xi,xf,acx,tf,dt);
    [y,ty] = Trapezoidal_Speed_Calc(yi,yf,acy,tf,dt);
    [z,tz] = Trapezoidal_Speed_Calc(zi,zf,acz,tf,dt);
    tx = tx';
    ty = ty';
    tz = tz';
%     plot(tx,x);
%     figure();
%     plot(ty,y);
%     figure();
%     plot(tz,z);
    dx = diff(x);
    dx(end+1) = 0;
    dy = diff(y);
    dy(end+1) = 0;
    dz = diff(z);
    dz(end+1) = 0;
    ddx = diff(dx);
    ddx(end+1) = ddx(end);
    ddy = diff(dy);
    ddy(end+1) = ddy(end);
    ddz = diff(dz);
    ddz(end+1) = ddz(end);
end