function C = waypoints(boundary, ps, ts)
    
    % example input: trajectory from r=0 @ t=0 to
    % r=1 @ t=5, with all higher boundary derivatives
    % equal to zero, with waypoints r=2 @ t=1, and
    % r=3 @ t=3
    % boundary = [0; 0; 0; 0; 1; 0; 0; 0];
    % ps = [2; 3];
    % ts = [1; 3; 5];
    
    % your implementation here...
    
    m = size(ts, 1); %# of segments
    time = [0; ts];
    pos = [boundary(1); ps; boundary(end)];
    
    
    a = sym('a', [8 m]); %mth column contains 8 coefficients of the mth segment polynomial
    
    
    syms D(t) 
    D(t) = derivative_matrix(7);
    
    C_t0 = sym(zeros(7,m)); %mth column contains the values of 0th through 6th derivatives of trajectory at t = 0
    d0 = subs(D(t),t,0);
    for index = 1:m
        C_t0(:,index) = d0*a(index,:);
    end
    C_t0(5:7,1) = sym([0;0;0]);
    
    C_tT = sym(zeros(7,m)); %mth column contains the values of 0th through 6th derivatives of trajectory at t = T
    for index = 1:m
        C_tT(:,index) = (subs(D(t),t,ts(index)))*a(index,:);
    end
    C_tT(5:7,m) = sym([0;0;0]);
    
    eqnmatrix = sym(zeros(m+1,1));
    eqnmatrix(1) = C_t0(1:4,1) == boundary(1:4,1);
    eqnmatrix(m+1) = C_tT(1:4,1) == boundary(5:8,1);
    for index = 1:m
        eqnmatrix(index+1) = C_tT(2:end,index) == C_t0(2:end,index + 1);
    end
    csol = solve(eqnmatrix,a);
    C = csol;
    
end

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