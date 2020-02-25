function grabObj()
initialize();
[colors, locations, sizes] = findObjs();
disp(locations); 
[T0_check, T_cam_to_checker] = camRobotRegistration();

TworldPoints = transpose(locations); 
% disp('T0 to check');
% disp(pinv(T_check_to_robot));
% disp('T-check to robot')
% disp(T_check_to_robot);

roboPoints = T0_check* [TworldPoints;   15 ;  1];
disp('roboPoints'); 
disp(roboPoints);
end 

% %
%    25.4717 -218.3438
%    69.0577  248.4448
%   -49.5000  -49.5000
%     1.0000    1.0000