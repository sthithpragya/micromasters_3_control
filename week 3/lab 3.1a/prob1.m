% number of links to consider
n = 2;

% DH parameters
syms a1 d1 d2 q1 q2 real

% the center of mass of each link measured relative to the link fixed frame
% (e.g. c1 = [c1x c1y c1z]' is measured relative to x1y1z1)
c = cell(n,1);
c{1} = [sym('c1x'); sym('c1y'); sym('c1z')];
c{2} = [sym('c2x'); sym('c2y'); sym('c2z')];
assume(vertcat(c{:}), 'real');

% initial conditions for the configuration of Sawyer shown in Figure 1.
% HINT: double(subs(expr, vars, vals)) evaluates a symbolic expression 'expr' by
% substituting each element of 'vals' with its corresponding symbolic variable in 'vars'
q0 = [0 3*pi/2]; % [q10 q20] mm
d0 = [317 192.5]; % [d10 d20] mm
a10 = 81; % in mm

% cell array of your homogeneous transformations; each Ti{i} is a 4x4 symbolic 
% transformation matrix in terms of the given DH parameters that transforms objects
% in frame i to the inertial frame 0
Ti = cell(n,1);

% The angular velocity Jacobian as an nx1 cell array where each element, Jw{i} is 
% a 3xn symbolic matrix
Jw = cell(n,1);

% The linear velocity Jacobian as an nx1 cell array where each element, Jv{i} is 
% a 3xn symbolic matrix
Jv = cell(n,1);

q = [q1; q2];
d = [d1; d2];
a = [a1; 0];

vars = [q;d;a];
vals = [q0.';d0.';a10;0];
alpha = [-pi/2; -pi/2];

Rz(q1) = [cos(q1) -sin(q1) 0 0;
    sin(q1) cos(q1) 0 0;
    0 0 1 0;
    0 0 0 1];

Pz(d1) = [1 0 0 0;
    0 1 0 0;
    0 0 1 d1;
    0 0 0 1];

Px(d1) = [1 0 0 d1;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

Rx(q1) = [1 0 0 0;
    0 cos(q1) -sin(q1) 0;
    0 sin(q1) cos(q1) 0;
    0 0 0 1];

% homogeneous transformations
for i = 1:n
    if i == 1
        ti = Rz(q(i))*Pz(d(i))*Px(a(i))*Rx(alpha(i));
        Ti{i} = ti;
    else
        ti = Rz(q(i))*Pz(d(i))*Px(a(i))*Rx(alpha(i));
        Ti{i} = Ti{i-1}*ti;
    end
end

% angular velocity Jacobian
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
    J = sym(zeros(3,n));
    for j = 1:i
        J(:,j) = zaxis(:,j);
    end
    Jw{i} = J;
end

% linear velocity Jacobian
for i = 1:n
    T = Ti{i};
    R = T(1:3,1:3);
    p_com = T(1:3,4) + R*c{i};
    jv = sym(zeros(3,n));
    for j = 1:i
        if j == 1
            jv(:,j) = cross(zaxis(:,j),p_com);
        else
            T = Ti{j-1};
            jv(:,j) = cross(zaxis(:,j),p_com - T(1:3,4));
        end
    end
    Jv{i} = jv;
end