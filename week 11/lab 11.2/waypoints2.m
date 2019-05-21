function C = waypoints2(boundary, ps, ts)
    
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
    pos = [boundary(1); ps; boundary(5)];
    
    
    a = sym('a', [8 m]); %mth column contains 8 coefficients of the mth segment polynomial
    
    syms D(t) 
    D(t) = derivative_matrix(7);
    
    C_t0 = sym(zeros(7,m)); %mth column contains the values of 0th through 6th derivatives of trajectory at t = 0
    d0 = subs(D(t),t,0);
    for index = 1:m
        C_t0(:,index) = d0*a(:,index);
    end
    %C_t0(5:7,1) = sym([0;0;0]);
    
    C_tT = sym(zeros(7,m)); %mth column contains the values of 0th through 6th derivatives of trajectory at t = T
    C_tT(:,1) = (subs(D(t),t,ts(1)))*a(:,1);
    for index = 2:m
        C_tT(:,index) = (subs(D(t),t,ts(index)-ts(index-1)))*a(:,index);
    end
    %C_tT(5:7,m) = sym([0;0;0]);
    
    %equating 0th order derivatives at t0
    eqn00 = sym(zeros(m,1));
    eqn0T = sym(zeros(m,1));
    for index = 1:m
        eqn00(index) = C_t0(1,index) == pos(index);
        eqn0T(index) = C_tT(1,index) == pos(index+1);
    end
    eqn0 = [eqn00; eqn0T];
    
    %equating 1st order derivatives
    eqn1 = sym(zeros(m+1,1));
    eqn1(1) = C_t0(2,1) == boundary(2);
    eqn1(end) = C_tT(2,end) == boundary(6);
    for index = 2:m
        eqn1(index) = C_tT(2,index-1) == C_t0(2,index);
    end
    
    %equating 2nd order derivatives
    eqn2 = sym(zeros(m+1,1));
    eqn2(1) = C_t0(3,1) == boundary(3);
    eqn2(end) = C_tT(3,end) == boundary(7);
    for index = 2:m
        eqn2(index) = C_tT(3,index-1) == C_t0(3,index);
    end
    
    %equating 3rd order derivatives
    eqn3 = sym(zeros(m+1,1));
    eqn3(1) = C_t0(4,1) == boundary(4);
    eqn3(end) = C_tT(4,end) == boundary(8);
    for index = 2:m
        eqn3(index) = C_tT(4,index-1) == C_t0(4,index);
    end

    %equating 4th order derivatives
    eqn4 = sym(zeros(m-1,1));
    for index = 1:m-1
        eqn4(index) = C_tT(5,index) == C_t0(5,index+1);
    end
    
    %equating 5th order derivatives
    eqn5 = sym(zeros(m-1,1));
    for index = 1:m-1
        eqn5(index) = C_tT(6,index) == C_t0(6,index+1);
    end
    
    %equating 6th order derivatives
    eqn6 = sym(zeros(m-1,1));
    for index = 1:m-1
        eqn6(index) = C_tT(7,index) == C_t0(7,index+1);
    end

    eqn = [eqn0; eqn1; eqn2; eqn3; eqn4; eqn5; eqn6];
    csol = struct2cell(solve(eqn,a));
    c = sym(zeros(8,m));
    for col = 1:m
        c(:, col) = [csol{8*(col-1)+1:8*col}];
    end
    C = double(c);
    double((subs(D(t),t,0))*C(:,1))
    
    double((subs(D(t),t,ts(1)))*C(:,1))
    double((subs(D(t),t,0))*C(:,2))
    
    double((subs(D(t),t,ts(2)-ts(1)))*C(:,2))
    double((subs(D(t),t,0))*C(:,3))
    
    double((subs(D(t),t,ts(3)-ts(2)))*C(:,end))
    
    
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