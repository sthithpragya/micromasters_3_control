%% homogeneous transforms

n = 7; % DOF

% DH parameters
q = sym('q', [n 1], 'real'); % generalized coordinates (joint angles)
d = sym('d', [n 1], 'real'); % link offsets
syms a1

syms g real % gravity

% initial conditions for the configuration of Sawyer shown in Figure 1.
% you can use these values to sense check your work
% HINT: vpa(subs(expr, vars, vals)) evaluates a symbolic expression 'expr' by
% substituting each element of 'vals' with its corresponding symbolic variable in 'vars'
q0 = [0 3*pi/2 0 pi 0 pi 3*pi/2];
d0 = [317 192.5 400 168.5 400 136.3 133.75];
a10 = 81;

% cell array of your homogeneous transformations; each Ti{i} is a 4x4 symbolic transform matrix
% provide your answer in terms of the given symbolic variables
% NOTE: for symbolic arrays: q(1) = q1, q(2) = q2, etc.
Ti = cell(n,1);

% Ti = your homogeneous transformations solution

%% angular velocity jacobian (Jw)

% Initialize angular velocity jacobian as an nx1 cell array where each element is
% an 3xn symbolic matrix
Jw = arrayfun(@(x) sym(['Jw' num2str(x)], [3,n], 'real'), 1:n, 'UniformOutput', 0)';

% Jw = your angular velocity jacobian solution

%% linear velocity jacobian (Jv)

% the center of mass of each link measured relative to the link fixed frame
% like Ti and Jw, c is an nx1 cell array where each element is a symoblic vector/matrix
% for example: c{3} = [c3x c3y c3z]' is the center of mass of link 3 measured relative to frame 3
c = arrayfun(@(x) [sym(['c' num2str(x) 'x'], 'real'), sym(['c' num2str(x) 'y'], 'real'), ...
    sym(['c' num2str(x) 'z'], 'real')]', 1:n, 'UniformOutput', 0)';

% as with the angular velocity jacobian, the linear velocity jacobian is comprised of n 3xn
% symbolic matrices stored in a cell array. Jv{i} is the 3xn angular velocity jacobian of link i
Jv = cell(n,1);

%  Jv = your linear velocity jacobian solution

%% potential energy

m = sym('m', [n 1], 'real'); % mass of each link

% PE = your potential energy solution

%% inertial matrix and kinetic energy

qd = sym('qd', [n 1], 'real'); % "q d" - the first derivative of the q's in time (joint velocities)

% inertia tensor for each link relative to the inertial frame stored in an nx1 cell array
I = arrayfun(@(x) inertia_tensor(x), 1:n, 'UniformOutput', 0)';

% D = your inertia matrix solution

% KE = your kinetic energy solution

%% equations of motion

qdd = sym('qdd', [n 1], 'real'); % "q double d" - the second derivative of the q's in time (joint accelerations)

a = [a1; 0; 0; 0; 0; 0; 0];
alpha = [-pi/2; -pi/2; -pi/2; -pi/2; -pi/2; -pi/2; 0];

Rz(a1) = [cos(a1) -sin(a1) 0 0;
    sin(a1) cos(a1) 0 0;
    0 0 1 0;
    0 0 0 1];

Pz(a1) = [1 0 0 0;
    0 1 0 0;
    0 0 1 a1;
    0 0 0 1];

Px(a1) = [1 0 0 a1;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

Rx(a1) = [1 0 0 0;
    0 cos(a1) -sin(a1) 0;
    0 sin(a1) cos(a1) 0;
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

PE = sym(0);
grav = [0;0;g];
for i = 1:n
    T = Ti{i};
    R = T(1:3,1:3);
    p_com = T(1:3,4) + R*c{i};
    PE = PE + m(i)*grav.'*p_com;
end

% the inertia matrix
D = sym(zeros(n,n));
for i = 1:n
    D = D + m(i)*Jv{i}.'*Jv{i} + Jw{i}.'*I{i}*Jw{i};
end
    
% kinetic energy
KE = 0.5*qd.'*D*qd;

% eom_lhs = your solution
C = sym(zeros(n,n));
for j = 1:n
    for k = 1:n
        for i = 1:n
            C(j,k) = C(j,k) + 0.5*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)))*qd(i);
        end
    end
end

G = sym(zeros(n,1));
for k = 1:n
    G(k) = diff(PE, q(k));
end

eom_lhs = D*qdd + C*qd + G; 
gradeEOM