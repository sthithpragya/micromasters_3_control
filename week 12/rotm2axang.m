function [axang] = rotm2axang(R) 
   
    t = trace(R);
    if t == 3
        theta = 0;
        vec1 = NaN(1,3);
        axang = [vec1, theta];
    elseif t == -1
        theta = pi;
        a = sqrt((R(1,1)+1)/2);
        b = sqrt((R(2,2)+1)/2);
        c = sqrt((R(3,3)+1)/2);
        vec = zeros(2,3);
        if a == 0
            if R(2,3) < 0
                vec = [a b -c; -a -b c];
            else
                vec = [a b c; -a -b -c];
            end
        else
            if b == 0
                if R(1,3) < 0
                    vec = [a b -c; -a -b c];
                else
                    vec = [a b c; -a -b -c];
                end
            else
                if c == 0
                    if R(1,2) < 0
                        vec = [a -b c; -a b -c];
                    else
                        vec = [a b c; -a -b -c];
                    end
                else
                    if R(1,2) > 0
                        if R(1,3) > 0 
                            vec = [a b c; -a -b -c];
                        else
                            vec = [a b -c; -a -b c];
                        end
                    else
                        if R(1,3) > 0 
                            vec = [a -b c; -a b -c];
                        else
                            vec = [a -b -c; -a b c];
                        end
                    end
                end
            end
        end
        axang = [vec(1,:) theta;
        vec(2,:) theta;];
    else
        theta = acos((trace(R)-1)/2);
        vec3(1,1) = (R(3,2)-R(2,3))/(2*sin(theta));
        vec3(1,2) = (R(1,3)-R(3,1))/(2*sin(theta));
        vec3(1,3) = (R(2,1)-R(1,2))/(2*sin(theta));
        axang = [vec3, theta];
    end
end