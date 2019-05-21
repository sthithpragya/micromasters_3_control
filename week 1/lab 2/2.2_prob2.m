% qd1 and qd2 are the time derivatives of q1 and q2 respectively
syms a1 d1 d2 q1 q2 c1 c2 c3 qd1 qd2 real

% provide your answer in terms of the above syms
Rz(q1) = [cos(q1) -sin(q1) 0 0;
    sin(q1) cos(q1) 0 0;
    0 0 1 0;
    0 0 0 1];

Pz(d1) = [1 0 0 0;
    0 1 0 0;
    0 0 1 d1;
    0 0 0 1];

Px(d1) = [1 0 0 d1;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

Rx(q1) = [1 0 0 0;
    0 cos(q1) -sin(q1) 0;
    0 sin(q1) cos(q1) 0;
    0 0 0 1];

T01 = Rz(q1)*Pz(d1)*Px(a1)*Rx(-pi/2);

T12 = Rz(q2)*Pz(d2)*Px(0)*Rx(-pi/2);

T02 = T01*T12;

pos1 = T01(1:3,4) + T01(1:3,1:3)*[0;0;c1];
w1 = qd1*[0;0;1];
vp1 = cross(w1,pos1); %using the traditional way
vp1_2 = diff(pos1,q1)*qd1; %using the differential derivative method

pos2 = T02(1:3,4) + T02(1:3,1:3)*[0;-c3;c2];
vp2 = diff(pos2,q1)*qd1+diff(pos2,q2)*qd2;

pos2_2 = T02(1:3,1:3)*[0;-c3;c2];
w2_2 = w1+T01(1:3,1:3)*qd2*[0;0;1];
vp2_2 = cross(w1,T02(1:3,4))+cross(w2_2,pos2_2);

isequal(vp1,vp1_2)
isequal(vp2,vp2_2)




