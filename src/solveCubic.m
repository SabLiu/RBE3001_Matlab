% Takes in 4x1 coefficients matrix for one joint at one edge
% Takes in t, which is tf-to for that edge
% returns 20 q values
function encoderPositions = solveCubic(a)
encoderPositions = zeros(1, 60); 

for i = 0:59
    t = i*0.025; 
    q = a(1) + a(2)*t + a(3)*(t^2) + a(4)*(t^3); 
    encoderPositions(1, i+1) = q; 
end 
end