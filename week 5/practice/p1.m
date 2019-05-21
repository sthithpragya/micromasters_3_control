% Sum1 = sumblk('e = r - y');
% Sum2 = sumblk('u = uC + uF');
% 
% % Define block I/Os ("u" and "y" are shorthand for "InputName" and "OutputName")
% F.u = 'r';   F.y = 'uF';
% C.u = 'e';   C.y = 'uC';
% G.u = 'u';   G.y = 'ym';
% S.u = 'ym';  S.y = 'y';
% 
% % Compute transfer r -> ym
% T = connect(F,C,G,S,Sum1,Sum2,'r','ym');
% step(T)

s = tf('s');
F = 1/(s+1);
G = 100/(s^2+5*s+100);
C = 20*(s^2+s+60)/s/(s^2+40*s+400);
S = 10/(s+10);
T = F * feedback(G*C,S);

step(10*T); %plots for step response
stepinfo(10*T) % calculates paramters like peak time etc
% pidTuner(T,'pd'); %tuning Kp Ki and Kd values


