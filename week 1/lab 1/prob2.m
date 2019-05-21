syms a1 d1 d2 q1 q2

% provide your answer in terms of the symbolic variables above

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

T01 = Rz(q1)*Pz(d1)*Px(a1)*Rx(-pi/2);

T12 = Rz(q2)*Pz(d2)*Px(0)*Rx(-pi/2);

T02 = T01*T12;

q = [q1; q2];
d = [d1; d2];

vars = [q;d;a1];
vals = [q0.';d0.';a10];

double(subs(T02, vars, vals))





