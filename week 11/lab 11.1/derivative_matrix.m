function D = derivative_matrix(n)
    syms t
    A = sym(zeros(n,8));
    for j = 1:n
        if j == 1
            for i = 1:8
                A(j,i) = t^(i-1);
            end
        else
            for i = 1:8
                A(j,i) = diff(A(j-1,i));
            end
        end
    end
    
    D = A;      
    % your implementation...
end