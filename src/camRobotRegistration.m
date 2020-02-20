% ROBOT-CAMERA registration (step 3)
%% Setup

serverParams = initialize(); 
PID_SERVER = serverParams(1); 
STATUS_SERVER = serverParams(2); 
clear('cam');
cam = webcam(); 
L1 = 135;
L2 = 175;
L3 = 169.28;
%% Calculate T0 to checker 
T01 = tdh(pi/2,-34.5,0,pi/2); %tdh(pi/2,0,0,pi/2); %
T02 = T01*tdh(0, L2+52.8+48, 101.6+12, 0); 
T03 = T02*tdh(0,0,0,pi/2); 
T04 = T03*tdh(-pi/2, 0,0,0); 
T0_check = T04; 
T_check_to_robot = inv(T0_check);

T_cam_to_checker = getCamToCheckerboard(cam, cameraParams); 
Tcheck_cam = pinv(T_cam_to_checker);
T0_cam =T0_check*Tcheck_cam; 

Pixel_values =   [308.3689  279.6416;
   65.7459  445.1352;
  127.3852  169.0837;
  483.4508  169.7704];
% [307.6584  279.5625;
%    65.4024  443.3125;
%   127.1811  170.8125;
%   483.2766  169.5625];

% Tcam_0 = pinv(T0_cam);
% worldPoints = pointsToWorld(cameraParams, Tcam_0(1:3,1:3), Tcam_0(1:3,end), Pixel_values);

worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), Pixel_values);
disp('pixel to checkerboard ref');
disp(worldPoints); 


% disp(T_check_to_robot);

TworldPoints = transpose(worldPoints); 
roboPoints = T0_check * [TworldPoints; 15 15 15 15; 1 1 1 1];
disp(roboPoints);

% origImg = snapshot(cam);
% img = undistortImage(origImg, cameraParams, 'OutputView', 'full');
% image(origImg)
% 
% disp(ginput(4)); 
% % 
% grid on

% second time pixel after 2nd calibration
%   312.4844  278.6011
%    70.1081  449.8989
%   127.1967  172.3138
%   486.7551  164.0585
  
% points wrt robot 0 - wrt checkerboard
%(175,0) - (100.8, -113.6)
%(279.5,-139) - (0, -249)
%(70.5,-139) - (209, -242)
%(70.5,139) - (209, 36)

