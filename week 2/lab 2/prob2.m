syms m h r z theta R

pos = [R*cos(theta); R*sin(theta); z];
diadic = (z^2+R^2)*eye(3,3) - pos*pos.';
i1 = int(R*diadic, R, 0, (z*r)/h);
i2 = int(i1, theta, 0, 2*pi);
i3 = int(i2, z, 0, h);
Ip(m, h, r) = i3*((3*m)/(pi*h*r^2));
% syms m h r x y z
% 
% pos = [x; y; z]
% diadic = (x^2+y^2+z^2)*eye(3,3) - pos*pos.'
% i1 = int(diadic, x, -sqrt((z*r/h)^2-y^2), sqrt((z*r/h)^2-y^2))
% i2 = int(i1, y, -r*z/h, r*z/h)
% i3 = int(i2, z, 0, h)
% Ip(m, h, r) = i3*((3*m)/(pi*h*r^2))