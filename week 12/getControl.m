function [f,M] = getControl(m,J,s,traj,t)
    r = s(1:3);
    v = s(4:6);
    w = s(16:18);
    g = 9.8;
    % world frame in quadrotor frame
    R_qw = [s(7:9)';s(10:12)';s(13:15)'];
    R = R_qw';
    
    KV = [0.5 0 0;
        0 0.5 0;
        0 0 0.5];
    
    KP = [15 0 0;
        0 15 0;
        0 0 15];
    
    
    KR = [10^6 0 0;
        0 10^6 0;
        0 0 10^6];
    
    KW = [25 0 0;
        0 25 0;
        0 0 25];
    
    x_des_coeff = traj(:,1);
    y_des_coeff = traj(:,2);
    z_des_coeff = traj(:,3);
    yaw_des_coeff = traj(:,4);
    
    x_des = 0;
    y_des = 0;
    z_des = 0;
    yaw_des = 0;
    
    
    for i = 1:8
        x_des = x_des + x_des_coeff(i)*(t^(i-1));
        y_des = y_des + y_des_coeff(i)*(t^(i-1));
        z_des = z_des + z_des_coeff(i)*(t^(i-1));
        yaw_des = yaw_des + yaw_des_coeff(i)*(t^(i-1));
    end
    
    x_des_dot = 0;
    y_des_dot = 0;
    z_des_dot = 0;
    yaw_des_dot = 0;
    
    for i = 2:8
        x_des_dot = x_des_dot + x_des_coeff(i)*(i-1)*(t^(i-2));
        y_des_dot = y_des_dot + y_des_coeff(i)*(i-1)*(t^(i-2));
        z_des_dot = z_des_dot + z_des_coeff(i)*(i-1)*(t^(i-2));
        yaw_des_dot = yaw_des_dot + yaw_des_coeff(i)*(i-1)*(t^(i-2));
    end
    x_des_ddot = 0;
    y_des_ddot = 0;
    z_des_ddot = 0;
    
    for i = 3:8
        x_des_ddot = x_des_ddot + x_des_coeff(i)*(i-1)*(i-2)*(t^(i-3));
        y_des_ddot = y_des_ddot + y_des_coeff(i)*(i-1)*(i-2)*(t^(i-3));
        z_des_ddot = z_des_ddot + z_des_coeff(i)*(i-1)*(i-2)*(t^(i-3));
    end
    
    r_des = [x_des; y_des; z_des];
    r_des_dot = [x_des_dot; y_des_dot; z_des_dot];
    r_des_ddot = [x_des_ddot; y_des_ddot; z_des_ddot];
            
    e_p = r - r_des;
    e_v = v - r_des_dot;
    F_des = -KP*e_p - KV*e_v + m*g*[0;0;1] + m*r_des_ddot;
    u1 = dot(F_des,(R*[0;0;1]));
    
    zB_des = F_des/norm(F_des);
    xC_des = [cos(yaw_des); sin(yaw_des); 0];
    yB_des = cross(zB_des, xC_des)/norm(cross(zB_des, xC_des));
    xB_des = cross(yB_des, zB_des);
    R_des = [xB_des yB_des zB_des];
            
    e_R = (1/2)*(R_des'*R - R'*R_des); 
    e_Rv = [e_R(3,2); e_R(1,3); e_R(2,1)];
    e_w = w - R'*R_des*[0;0;yaw_des_dot];
    
    u2 = cross(w, J*w) + J*(-KR*e_Rv - KW*e_w);
    %u2 = -KR*e_Rv - KW*e_w;
    
    f = u1;
    M = u2;

end