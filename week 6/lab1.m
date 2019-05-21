syms d11 d12 d21 d22 c121 c211 c221 c112 g1 g2 tau1 tau2 x1 x2 x3 x4 x2d x4d real 

eqn1 = d11*x2d + d12*x4d + c121*x2*x4 + c211*x2*x4 + c221*x4*x4 + g1 == tau1;
eqn2 = d21*x2d + d22*x4d + c112*x2*x2 + g2 == tau2;
eqns = [eqn1, eqn2];
vars = [x2d, x4d];

[X2d, X4d] = solve(eqns, vars);

f_x_tau = [x2; X2d; x4; X4d]; 