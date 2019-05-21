A = [1 -3; -1 -2];
[P, lambda] = eig(A);
B = [1; -2];
C = [1 1];
CM = ctrb(A,B);
det(CM);
OM = obsv(A,C);
det(OM);
R = 3;
Q = C'*C;
[p, L, G] = care(A, B, Q, R);
L
K = inv(R)*B'*p
k = [156/17 -21/17];
Ap  = eig(A - B*K);
ap  = eig(A - B*k);
round(k,2);
