% Take in 6x1 coefficients matrix e
% Return encoder positions for 1 joint for 1 edge

function a = quinticPositions(e)
encoderPositions = zeros(); 

for i = 0:19
    t = i*0.1; 
    q = e(1) + t*e(2) + (t^2)*e(3) + (t^3)*e(5) + (t^4)*e(6);
    encoderPositions = [encoderPositions q]; % append to the right 
end
end