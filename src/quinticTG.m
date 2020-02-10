
% takes in to, tf, qo, qf, vo, vf, ao, af
% returns 6x1 of coefficients for trajectory generation with quintic poly
% need to run this function on each joint at each vertex: 9 times

function e = quinticTG(to, tf, qo, qf, vo, vf, ao, af)
    % Quintic time matrix
    T = [ 1 to (to)^2 (to)^3 (to)^4 (to)^5;
        0 1 2*to 3*((to)^2) 4*((to)^3) 5*((to)^4);
        0 0 2 6*(to) 12*((to)^2) 20*((to)^3);
        1 tf (tf)^2  (tf)^3 (tf)^4 (tf)^5;
        0 1 2*(tf) 3*((tf)^2) 4*((tf)^3) 5*((tf)^4);
        0 0 2 6*(tf) 12*((tf)^2) 20*((tf)^3)]; 
    % Right hand side parameters
    q = [qo; vo; ao; qf; vf; af];
    % solves the system of equations Tx = q for x 
    e = T\q;
end

