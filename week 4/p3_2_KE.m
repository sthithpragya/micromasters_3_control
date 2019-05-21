n = 7; % degrees of freedom of Sawyer

% initial conditions for the configuration of Sawyer shown in Figure 1.
% you can use these values to sense check your work
% HINT: vpa(subs(expr, vars, vals)) evaluates a symbolic expression 'expr' by
% substituting each element of 'vals' with its corresponding symbolic variable in 'vars'
q0 = [0 3*pi/2 0 pi 0 pi 3*pi/2];
d0 = [317 192.5 400 168.5 400 136.3 133.75];
a10 = 81;

% symbolic variables
q  = sym('q',  [n 1], 'real'); % generalized coordinates (joint angles)
qd = sym('qd', [n 1], 'real'); % "q dot" - the first derivative of the q's in time (joint velocities)
d  = sym('d',  [n 1], 'real'); % link offsets
m  = sym('m',  [n 1], 'real'); % mass of each link
syms a1

% Jw, Jv - from previous problems
load('angular_velocity_jacobian.mat');
load('linear_velocity_jacobian.mat');

% inertia tensor for each link relative to the inertial frame stored in an nx1 cell array
I = arrayfun(@(x) inertia_tensor(x), 1:n, 'UniformOutput', 0)';
%------------------------------------------------------------------------------------
% the inertia matrix
D = sym(zeros(n,n));
for i = 1:n
    D = D + m(i)*Jv{i}.'*Jv{i} + Jw{i}.'*I{i}*Jw{i};
end
    
% kinetic energy
KE = 0.5*qd.'*D*qd;