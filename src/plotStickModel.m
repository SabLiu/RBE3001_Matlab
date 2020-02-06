% q : 3x1 with joint angles (thetas)
function B = plotStickModel(q)
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
robotLinks = plot3(X,Y,Z);
hold on
% Plot joint points
robotPoints = plot3(X,Y,Z,'r.');
grid on
axis([0 300 -300 300 -50 300]);
% Pause so graph is visible
pause(0.197); % 0.2 - 0.003
delete(robotPoints);
delete(robotLinks);
B = P3;
end
