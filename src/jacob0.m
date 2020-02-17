% takes in q: 3 thetas in 3x1 matrix
% World Reference Frame: 0 or WRF
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

% WRF to first joint 
% They are in the same spot, so T is identity matrix
T01 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];
p01 = T01(1:3, 4); % this is essentially a 0 vector
z1 = T01(1:3, 3); % this is [0;0;1] since z's are on top of each other

% First joint to second joint
T12 = tdh(-q1, L1, 0, pi/2);
% WRF to second joint
T02 = T01*T12;
p02 = T02(1:3,4);
z2 = T02(1:3,3);

% 2nd joint to 3rd joint
T23 = tdh(q2 , 0, L2, 0);
% WRF to 3rd joint
T03 = T02*T23;
z3 = T03(1:3, 3);
p03 = T03(1:3, 4);

% Calculate cross products for Jp
% Plug in zi for Jo
j = [cross(z1, pe-p01) cross(z2, (pe-p02)) cross(z3, (pe-p03));
    z1 z2 z3]; 
    
% j = [-L3*sin(q1)*cos(q2)*sin(q3)-L3*sin(q1)*sin(q2)*cos(q3) -L3*cos(q1)*sin(q2)*sin(q3)+L3*cos(q1)*cos(q2)*cos(q3) L3*cos(q1)*cos(q2)*cos(q3)-L3*cos(q1)*sin(q2)*sin(q3);
%      L3*cos(q1)*cos(q2)*sin(q3)+L3*cos(q1)*sin(q2)*cos(q3) -L3*sin(q1)*sin(q2)*sin(q3)+L3*sin(q1)*cos(q2)*cos(q3) L3*sin(q1)*cos(q2)*cos(q3)-L3*sin(q1)*sin(q2)*sin(q3);
%      0 L3*cos(q2)*sin(q3)-L3*sin(q2)*cos(q3) L3*sin(q2)*cos(q3)+L3*cos(q2)*sin(q3);
%      sin(q1) sin(q1) -sin(q1);
%      -cos(q1) -cos(q1) cos(q1);
%      0 0 0]; 
end