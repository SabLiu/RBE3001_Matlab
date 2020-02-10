% Takes in 2 points, 1x3 vecotrs
function l = linearInterpolation(P1, P2)
n = 20;
t = linspace(0,1,n)';
l = (1-t)*P1 + t*P2;
end
% returns n number of points between those 2 points (also 1x3 vectors)