% input: 3x1 xyz tip position
% returns set of corresponding joint angles
% Also has safety checks to make sure that the desired point lies
%  within the reachable workspace of the robot
function V = ikin(p)
px = p(1);
py = p(2);
pz = p(3); 
% if the desired position is out of bounds
%errorMsg = ' No. ';
%error(errorMsg); 

L1 = 135; 
L2 = 175; 
L3 = 169.28;

d=sqrt((px^2)+(py^2)+(pz-L1)^2);
alpha = asin((pz-L1)/d);
beta = acos(((L2^2)+(d^2)-(L3^2))/(2*L2*d));

theta1 = atan2(py,px);
theta2 = alpha + beta;
theta3 = acos(-((L2^2)+(L3^2)-(d^2))/(2*L2*L3));

V = [-theta1; theta2; pi - theta3];
% theta1 is in same direction as fwkin so keep it the same (negative)
% theta 2 and 3 are opposite to fwkin so flip their signs (make them
% positive)
end 