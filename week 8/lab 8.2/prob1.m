syms p a B real % p = rho, a = alpha, b = beta 
syms k_p k_a k_b real
x = [p; a; B];
f1 = [-cos(a) 0; sin(a)/p -1; -sin(a)/p 0];
f2 = [k_p 0 0; 0 k_a k_b];
f = f1*f2*x;
%linearised system
A = [diff(f,x(1)), diff(f,x(2)), diff(f,x(3))];
A1 = subs(A, x, [0; 0; 0]);
%gains
K = [1 2 -1];
subs(eig(A1), [k_p k_a k_b], [1 2 -1])
