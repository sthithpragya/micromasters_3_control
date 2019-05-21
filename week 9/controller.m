function x_dot = controller(t, state_vec) % x_dot = f(x)
    
    % sprayer offset
    l = 0.1;
    
    % the position and orientation of the robot
    x = state_vec(1);
    y = state_vec(2);
    theta = state_vec(3);
    
    %syms k1 k2
    xp = x + l*cos(theta);
    yp = y + l*sin(theta);
    
    xpd = 10*cos(pi*t/5) + 5*sin(pi*t/10);
    ypd = 10*sin(pi*t/10) - 5*cos(pi*t/10) + 5;
    
    xpddot = (-2*pi)*sin(pi*t/5) + (pi/2)*cos(pi*t/10);
    ypddot = pi*cos(pi*t/10) + (pi/2)*sin(pi*t/10);
    
    xpdot = xpddot - (1.75)*(xp - xpd); %k1 = 1.75
    ypdot = ypddot - (1.75)*(yp - ypd); %k2 = 1.75
    
    z = [cos(theta) -l*sin(theta);
        sin(theta) l*cos(theta)];
    
    input = inv(z)*[xpdot; ypdot];
    
    x_dot = [cos(theta) 0; sin(theta) 0; 0 1]*input;
    %x_dot = subs(x_dot, [k1 k2], [1 1]);
    x_dot = double(x_dot);
    % your code here
end