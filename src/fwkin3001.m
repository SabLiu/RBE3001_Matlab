function p = fwkin3001(theta1, theta2, theta3)
% Static link lengths for 3001 robot
L1 = 135;
L2 = 175;
L3 = 169.28;
% Find transformation matrices
T1 = tdh(-theta1, L1, 0, pi/2);
T2 = tdh(theta2 , 0, L2, 0);
T3 = tdh(theta3 -(pi/2), 0, L3, pi);
% Calculate transformation matrix from 0 to 3
T = T1*T2*T3;
% Only return the 3x1 position matrix
p = T(1:3, 4);
end