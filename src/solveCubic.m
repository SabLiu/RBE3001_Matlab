% Takes in 4x1 coefficients matrix for one joint at one edge
% Takes in t, which is tf-to for that edge
% returns single value p which is the x or y or z location in task space
function p = solveCubic(a, t)
p = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3; 
end