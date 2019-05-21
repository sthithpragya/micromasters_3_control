function [f,M] = getControlA(m,J,s,traj,t)
    f = 0;
    M = [0;0;0];
    
    r = s(1:3);
    v = s(4:6);
    w = s(16:18);
    g = 9.8;
    % world frame in quadrotor frame
    R_qw = [s(7:9)';s(10:12)';s(13:15)'];
    R = R_qw';
    
    syms T 
    Kv = 10; 
    Kp = 10;
    Kr = 100;
    Kw = 100;
    
    x_des_coeff = traj(:,1);
    y_des_coeff = traj(:,2);
    z_des_coeff = traj(:,3);
    yaw_des_coeff = traj(:,4);
    
    x_des = sym(0);
    y_des = sym(0);
    z_des = sym(0);
    yaw_des = sym(0);
    
    for i = 1:size(x_des_coeff,2)
        x_des = x_des + x_des_coeff(i)*T^(i-1);
        y_des = y_des + y_des_coeff(i)*T^(i-1);
        z_des = z_des + z_des_coeff(i)*T^(i-1);
        yaw_des = yaw_des + yaw_des_coeff(i)*T^(i-1);
    end
        
    r_des = [x_des; y_des; z_des];
    r_des_dot = diff(r_des, T);
    r_des_ddot = diff(r_des_dot, T);
    
    e_r = r - r_des;
    e_r_dot = v - r_des_dot;
    tval = r_des_ddot + Kv*e_r_dot + Kp*e_r + g*[0;0;1];
    u1 = m*dot(tval,(R*[0;0;1]));
    
    tmatrix = tval/sqrt(tval'*tval);
    a = tmatrix(1);
    b = tmatrix(2);
    c = tmatrix(3);
    phi = atan(a*sin(yaw_des) - b*cos(yaw_des), c);
    theta = atan(a*cos(yaw_des) + b*sin(yaw_des),c/cos(phi));
    
    Rzyaw = [cos(yaw_des) -sin(yaw_des) 0;
        sin(yaw_des) cos(yaw_des) 0;
        0 0 1];
    Rytheta = [cos(theta) 0 sin(theta);
        0 1 0;
        -sin(theta) 0 cos(theta)];
    Rxphi = [1 0 0;
        0 cos(phi) -sin(phi);
        0 sin(phi) cos(phi)];
    
    R_des = Rzyaw*Rytheta*Rxphi;
    
    %R_e_R = R_qw*R_des;
    %dR = rotm2axang(R_e_R);
    
    e_R = (1/2)*(R_des'*R - R'*R_des);    
    e_w_hat = inv(R_des'*R)*diff(R_des'*R, T);
    e_w = [e_w_hat(3,2); e_w_hat(1,3); e_w_hat(2,1)];
    
    %e_w_new = w - R'*R_des*[0;0;yaw_des];
    
    u2 = cross(w, J*w) + J*(-Kr*e_R - Kw*e_w);
    
    f = subs(u1, T, t);
    M = subs(u2, T, t);
end

% function [axang] = rotm2axang(R) 
%    
%     t = trace(R);
%     if t == 3
%         theta = 0;
%         vec1 = NaN(1,3);
%         axang = [vec1, theta];
%     elseif t == -1
%         theta = pi;
%         a = sqrt((R(1,1)+1)/2);
%         b = sqrt((R(2,2)+1)/2);
%         c = sqrt((R(3,3)+1)/2);
%         vec = zeros(2,3);
%         if a == 0
%             if R(2,3) < 0
%                 vec = [a b -c; -a -b c];
%             else
%                 vec = [a b c; -a -b -c];
%             end
%         else
%             if b == 0
%                 if R(1,3) < 0
%                     vec = [a b -c; -a -b c];
%                 else
%                     vec = [a b c; -a -b -c];
%                 end
%             else
%                 if c == 0
%                     if R(1,2) < 0
%                         vec = [a -b c; -a b -c];
%                     else
%                         vec = [a b c; -a -b -c];
%                     end
%                 else
%                     if R(1,2) > 0
%                         if R(1,3) > 0 
%                             vec = [a b c; -a -b -c];
%                         else
%                             vec = [a b -c; -a -b c];
%                         end
%                     else
%                         if R(1,3) > 0 
%                             vec = [a -b c; -a b -c];
%                         else
%                             vec = [a -b -c; -a b c];
%                         end
%                     end
%                 end
%             end
%         end
%         axang = [vec(1,:) theta;
%         vec(2,:) theta;];
%     else
%         theta = acos((trace(R)-1)/2);
%         vec3(1,1) = (R(3,2)-R(2,3))/(2*sin(theta));
%         vec3(1,2) = (R(1,3)-R(3,1))/(2*sin(theta));
%         vec3(1,3) = (R(2,1)-R(1,2))/(2*sin(theta));
%         axang = [vec3, theta];
%     end
% end



