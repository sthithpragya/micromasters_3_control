s = tf('s');

A = 10+0.15/s;
B = 1/(10*s^2+5*s);
C = 15*s;

Sum1 = sumblk('b = a - y');
Sum2 = sumblk('e = c - d');
% Sum3 = sumblk('g = e - f');


% Define block I/Os ("u" and "y" are shorthand for "InputName" and "OutputName")
A.u = 'b';
B.u = 'e';
C.u = 'y';

A.y = 'c';
B.y = 'y';
C.y = 'd';

% Compute transfer r -> ym
T = connect(A,B,C,Sum1,Sum2,'a','y');

step(T); %plots for step response
stepinfo(T) % calculates paramters like peak time etc
pidTuner(T,'pid'); %tuning Kp Ki and Kd values