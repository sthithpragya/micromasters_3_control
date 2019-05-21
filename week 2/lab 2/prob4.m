syms m h r a b

dist1 = [0; 0; -3*h/4];
Idiff1 = sym(zeros(3,3));
for i = 1:3
    for j = 1:3
        if i == j
            Idiff1(i,j) = m*(dist1.'*dist1-dist1(i)*dist1(j));
        else
            Idiff1(i,j) = m*(-dist1(i)*dist1(j));
        end
    end
end

dist2 = [-a; -b; -3*h/4];
Idiff2 = sym(zeros(3,3));
for i = 1:3
    for j = 1:3
        if i == j
            Idiff2(i,j) = m*(dist2.'*dist2-dist2(i)*dist2(j));
        else
            Idiff2(i,j) = m*(-dist2(i)*dist2(j));
        end
    end
end

% Io = Ip - Idiff1 + Idiff2;
Idiff = Idiff2 - Idiff1;