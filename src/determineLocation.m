% takes in color and size of current object. 
% returns X,Y position IN TASK SPACE that the robot should go to. 
function [X,Y] = determineLocation(color, size)
if (color == 1)
    X = 50;
elseif (color == 2)
    X = 150;
elseif (color == 3)
    X = 220;
end

% small 
if (size ==0)
    Y = -220;
else % big
    Y = 220; 
end
end
% Sorting scheme
% small     ______R____         large
% B        |          |         B
% G        |          |         G
% Y        |          |         Y
%          ____________