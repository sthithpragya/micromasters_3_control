syms m h
dist = [0; 0; -3*h/4];
Idiff = sym(zeros(3,3));
for i = 1:3
    for j = 1:3
        if i == j
            Idiff(i,j) = m*(dist.'*dist-dist(i)*dist(j));
        else
            Idiff(i,j) = m*(-dist(i)*dist(j));
        end
    end
end

Idiff = -Idiff;
