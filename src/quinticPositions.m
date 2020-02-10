% Take in 6x1 coefficients matrix e
% Return 1x20 matrix, 20 encoder positions for 1 joint for 1 edge

% goal: take in coefficients, plug in times, get encoder values 
function encoderPositions = quinticPositions(e)
encoderPositions = zeros(1,20); 

for i = 0:19
    t = i*0.1; 
    q = e(1) + t*e(2) + (t^2)*e(3) + (t^3)*e(4) + (t^4)*e(5) + (t^5)*e(6);
    encoderPositions(1,i+1) = q; % append  
end
end