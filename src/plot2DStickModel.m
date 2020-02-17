% q is 3x1 joint angle thetas

function X = plot2DStickModel(q)
clf
% Static link lengths for 3001 robot
L1 = 135;
L2 = 175;
L3 = 169.28;
% Base
P0 = [0; 0; 0];
% Transformation matrices
T1 = tdh(-q(1), L1, 0, pi/2);
T2 = T1*tdh(q(2), 0, L2, 0);
T3 = T2*tdh(q(3)- pi/2, 0, L3, pi);
% Position vectors from the T matrices
P1 = T1(1:3, 4);
P2 = T2(1:3, 4);
P3 = T3(1:3, 4);
% Create a matrix of all the points
allPoints = [P0 P1 P2 P3];
% Create arrays for X,Y,Z to plot
X = allPoints(1,:);
Y = allPoints(2,:);
Z = allPoints(3,:);

% Create 3D plot
% Plot links
robotLinks = plot(X,Z);
hold on
% Plot joint points
robotPoints = plot(X,Z,'r.');
grid on
% don't need to delete the robot
xlim([-100 500]); 
ylim([-100 500]); 
B = P3;

end 