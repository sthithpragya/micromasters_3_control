syms x1 x2 k1 k2 s
X = [x1; x2];
A = [1 -3; -1 -2];
B = [1; -2];
K = [k1 k2];
eq = (s*eye(2,2) - A + B*K);
det(eq);

eqn = [k1 - 2*k2 -4 == 0, 8*k1 + k2 - 11 == 0];
kv = solve(eqn, K);
kval = struct2cell(kv);
