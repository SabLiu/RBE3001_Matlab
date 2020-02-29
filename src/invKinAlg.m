
% pd is desired position (3x1)
% q0 is current thetas (3x1) 
% qe is joint variables (thetas) needed to reach that position

function qi = invKinAlg(q0, pd)
errorFK = fwkin3001(q0(1), q0(2), q0(3)); 
error = pd - errorFK(1:3, 4);

qi = q0; 
count = 0;
while (norm(error) > 0.1)
    fw = fwkin3001(qi(1), qi(2), qi(3));
    fq0 = fw(1:3, 4);
    error = pd-fq0;
    j = jacob0(qi);
    deltaQ = pinv(j(1:3, 1:3))*error;
    deltaQ(1) = -deltaQ(1);
    qi = qi + deltaQ;
    count = count + 1;
end
disp(qi); 
n = fwkin3001(qi(1), qi(2), qi(3));
disp(n); 

end

