n = 7; % degrees of freedom of Sawyer

% initial conditions for the configuration of Sawyer shown in Figure 1.
% you can use these values to sense check your work
% HINT: vpa(subs(expr, vars, vals)) evaluates a symbolic expression 'expr' by
% substituting each element of 'vals' with its corresponding symbolic variable in 'vars'
q0 = [0 3*pi/2 0 pi 0 pi 3*pi/2];
d0 = [317 192.5 400 168.5 400 136.3 133.75];
a10 = 81;

% symbolic variables
q = sym('q', [n 1], 'real'); % generalized coordinates (joint angles)
d = sym('d', [n 1], 'real'); % link offsets
syms a1

% loads Ti - the homogeneous transformations solved for previously
%   load('transforms.mat');

% the center of mass of each link measured relative to the link fixed frame
% like Ti and Jw, c is an nx1 cell array where each element is a symoblic vector/matrix
% for example: c{3} = [c3x c3y c3z]' is the center of mass of link 3 measured relative to frame 3
c = arrayfun(@(x) [sym(['c' num2str(x) 'x'], 'real'), sym(['c' num2str(x) 'y'], 'real'), ...
    sym(['c' num2str(x) 'z'], 'real')]', 1:n, 'UniformOutput', 0)';

% as with the angular velocity jacobian, the linear velocity jacobian is comprised of n 3xn
% symbolic matrices stored in a cell array. Jv{i} is the 3xn angular velocity jacobian of link i
Jv = cell(n,1);
%--------------------------------------------------------------------------

zaxis = sym(zeros(3,n));
for i = 1:n
    if i == 1
        zaxis(:,i) = sym([0;0;1]);
    else
        T = Ti{i-1};
        zaxis(:,i) = T(1:3,3);
    end
end

for i = 1:n
    T = Ti{i};
    R = T(1:3,1:3);
    p_com = T(1:3,4) + R*c{i};
    jv = sym(zeros(3,n));
    for j = 1:i
        if j == 1
            jv(:,j) = cross(zaxis(:,j),p_com);
        else
            t = Ti{j-1};
            jv(:,j) = cross(zaxis(:,j),p_com - t(1:3,4));
        end
    end
    Jv{i} = jv;
end