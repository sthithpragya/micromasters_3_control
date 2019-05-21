s = tf('s');
T = (10*s+0.15)/(10*s^3 + 20*s^2 + 10*s + 0.2);

step(10*T); %plots for step response
% stepinfo(T) % calculates paramters like peak time etc
pidTuner(T,'pid'); %tuning Kp Ki and Kd values
