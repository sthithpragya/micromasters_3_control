function [f,M] = getControl3(m,J,s,traj,t)
    r = s(1:3);
    v = s(4:6);
    w = s(16:18);
    g = 9.8;
    % world frame in quadrotor frame
    R_qw = [s(7:9)';s(10:12)';s(13:15)'];
    R = R_qw';
    
    syms T 
    %Kp = 1000;
    %Kv = 250; 
    %Kr = 5000000;
    %Kw = 10000;
    
    KV = [50 0 0;
        0 50 0;
        0 0 25];
    
    KP = [750 0 0;
        0 750 0;
        0 0 300];
    
    
    KR = [1500 0 0;
        0 1500 0;
        0 0 1000];
    
    KW = [100 0 0;
        0 100 0;
        0 0 50];
    
    x_des_coeff = traj(:,1);
    y_des_coeff = traj(:,2);
    z_des_coeff = traj(:,3);
    yaw_des_coeff = traj(:,4);
    
    x_des = sym(0);
    y_des = sym(0);
    z_des = sym(0);
    yaw_des = sym(0);
    
    for i = 1:8
        x_des = x_des + x_des_coeff(i)*(T^(i-1));
        y_des = y_des + y_des_coeff(i)*(T^(i-1));
        z_des = z_des + z_des_coeff(i)*(T^(i-1));
        yaw_des = yaw_des + yaw_des_coeff(i)*(T^(i-1));
    end
    
    r_des = [x_des; y_des; z_des];
    r_des_dot = diff(r_des, T);
    r_des_ddot = diff(r_des_dot, T);
    yaw_des_dot = diff(yaw_des, T);
%     r_des_dddot = diff(r_des_ddot1, T);
    
    r_des = double(subs(r_des, T, t));
    r_des_dot = double(subs(r_des_dot, T, t));
    r_des_ddot = double(subs(r_des_ddot, T, t));
    yaw_des = double(subs(yaw_des, T, t));
    yaw_des_dot = double(subs(yaw_des_dot, T, t));

    e_p = r - r_des;
    e_v = v - r_des_dot;
    F_des = -KP*e_p - KV*e_v + m*g*[0;0;1] + m*r_des_ddot;
    u1 = dot(F_des,(R*[0;0;1]));
    
    zB_des = F_des/norm(F_des);
    xC_des = [cos(yaw_des); sin(yaw_des); 0];
    yB_des = cross(zB_des, xC_des)/norm(cross(zB_des, xC_des));
    xB_des = cross(yB_des, zB_des);
    R_des = [xB_des yB_des zB_des];
    
%     hw = (m/u1)*(r_des_dddot - dot([0;0;1],r_des_dddot)*[0;0;1]);
%     w_des = [-dot(hw,[0;1;0]); dot(hw,[1;0;0]); diff(yaw_des,T)];
        
    e_R = (1/2)*(R_des'*R - R'*R_des); 
    e_Rv = [e_R(3,2); e_R(1,3); e_R(2,1)];
    e_w = w - [0;0;yaw_des_dot];
%     e_w_2 = w - w_des
    
%     u2 = cross(w, J*w) + J*(-KR*e_Rv - KW*e_w);
    u2 = -KR*e_Rv - KW*e_w;
    
    f = u1
    M = u2
end