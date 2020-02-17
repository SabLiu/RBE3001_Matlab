

function errorMsg = singularityEStop(q)
J = jacob0(q);
Jp = J(1:3,1:3);
A = Jp*transpose(Jp);
eigA = sqrt(eig(A));
volume = (4/3 * pi) * eigA(1)*eigA(2)*eigA(3);
errorMsg = ' ';
% if getting closer to a singularity
if ((volume<= 2.4*10^7)&&(volume>1.5*10^7))
    errorMsg = 'WARNING!!! APPROACHING A SINGULARITY. ';
    text(-100,-100, errorMsg, 'Color', 'red');
% HIT A SINGULARITY. EMERGENCY STOP!
elseif (volume <= (1.5*10^7))
    errorMsg = 'E-STOP ACTIVATED.'; 
    error(errorMsg);
else 
    errorMsg = '  '; 
    clf  
end

end