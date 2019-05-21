B1 = [1 2 3; 3 2 1; 2 1 3];
eig(A);
B2 = [0 1 0; 0 0 1; -6 -11 -6];
eig(B);

syms t real
A = [0 1 0;0 0 1;-6 -11 -6];
x0 = [1 2 -1]';
[p, lambda] = eig(A);
n = size(lambda,1);
lambda = t*lambda;
for i = 1:n
    lambda(i,i) = exp(lambda(i,i));
end

x_t = (p*lambda*inv(p))*x0;