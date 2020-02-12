% takes in q: 3 thetas in 3x1 matrix
function j = jacob0(q)
q1 = q(1);
q2 = q(2);
q3 = q(3);

L1 = 135;
L2 = 175;
L3 = 169.28;

% we need pe, which comes from T0e. In our case, T03 (T 0 to 3)
T0e = fwkin3001(q1,q2,q3);
ze = T0e(1:3,3);
pe = T0e(1:3, 4);

% we need p0i, which comes from T0i
% we need zi, which also comes from T0i
T01 = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];%tdh(-q1, L1, 0, pi/2);
p01 = T01(1:3, 4);
z1 = T01(1:3, 3);

T12 = tdh(q1, L1, 0, pi/2);
T02 = T01*T12;
% T02 = T12;
p02 = T02(1:3,4);
z2 = T02(1:3,3);


T23 = tdh(q2 , 0, L2, 0); % fwkin3001(q1,q2,q3);
T03 = T02*T23;
z3 = T03(1:3, 3);
p03 = T03(1:3, 4);
disp(z1);

j = [cross(z1, pe-p01) cross(z2, (pe-p02)) cross(z3, (pe-p03));
    z1 z2 z3]; 
    
% j = [-L3*sin(q1)*cos(q2)*sin(q3)-L3*sin(q1)*sin(q2)*cos(q3) -L3*cos(q1)*sin(q2)*sin(q3)+L3*cos(q1)*cos(q2)*cos(q3) L3*cos(q1)*cos(q2)*cos(q3)-L3*cos(q1)*sin(q2)*sin(q3);
%      L3*cos(q1)*cos(q2)*sin(q3)+L3*cos(q1)*sin(q2)*cos(q3) -L3*sin(q1)*sin(q2)*sin(q3)+L3*sin(q1)*cos(q2)*cos(q3) L3*sin(q1)*cos(q2)*cos(q3)-L3*sin(q1)*sin(q2)*sin(q3);
%      0 L3*cos(q2)*sin(q3)-L3*sin(q2)*cos(q3) L3*sin(q2)*cos(q3)+L3*cos(q2)*sin(q3);
%      sin(q1) sin(q1) -sin(q1);
%      -cos(q1) -cos(q1) cos(q1);
%      0 0 0]; 
end