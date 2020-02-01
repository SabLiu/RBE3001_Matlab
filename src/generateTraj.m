% Takes in start/end: time, velocity, position
% Generates 4x1 array C with coefficients ai, i = 0,1,2,3 
% of the cubic polynomial trajectory. 
% [C] = [time matrix]^-1 * [q, v matrix]

function C = generateTraj(to, tf, vo, vf, qo, qf)
    % time matrix
    T = [ 1 to to^2 to^3;
        0 1 2*to 3*(to^2);
        1 tf tf^2 tf^3;
        0 1 2*tf 3*(tf^2)];
    % q and v values
    qv = [qo; vo; qf; vf];
    
    % A\b solves the system Ax = b for x
    C = T\qv;
end