n = 7; % DOF

% DH parameters
q = sym('q', [n 1], 'real'); % generalized coordinates (joint angles)
d = sym('d', [n 1], 'real'); % link offsets
syms a1

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
%-------------------------------------------------------------------------------------------------
% your code may look something like:

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