% Takes in 3x1 q of current joint angles
% Takes in 3x1 dq (q') of instantaneous joint velocities
% Returns 3x1 dp (p') of task-space velocities

function dp = fwdVel(q, dq)
dp = jacob0(q)*dq;
end