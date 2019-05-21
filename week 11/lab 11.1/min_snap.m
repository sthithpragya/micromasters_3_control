function coefficients = min_snap(T,boundary)
    % boundary is a column vector in the form
    % [r0; v0; a0; j0; rT; vT; aT; jT]
    % representing the derivatives of the trajectory
    % at time 0 and time T
    
    syms D(t) 
    D(t) = derivative_matrix(4);
    d1 = subs(D(t),t,0);
    d2 = subs(D(t),t,T);
    d = [d1; d2];
    coefficients = linsolve(d,boundary);
    % your implementation...

end