function [colors, locations, sizes] = findObjs()
% [T_checker_to_robot, T_cam_to_checker] = camRobotRegistration(); 
% disp('REMOVE CHECKER');
% pause(5); 

% FINDOBJS implements a sequence of image processing steps to detect
% any objects of interest that may be present in an RGB image.

%%   Outputs
%   IMDETECTEDOBJS - a binarized image showing the location of the
%   segmented objects of interest.
%   
%   ROBOTFRAMEPOSE - the coordinates of the objects expressed in the robot's
%   reference frame
% see https://www.mathworks.com/help/vision/ref/cameraparameters.pointstoworld.html
% for details on the expected dimensions for YOUR_PIXEL_VALUES)


%% Capture & Undistort the image 
cam = webcam(); 
imOrig = snapshot(cam); 
load camParams.mat; 
load 'TcamCheck.mat'
load 'T0check.mat'
img = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
imwrite(img, 'InputImage.png'); 

colors = ['blue'];

%%  Find the colors and locations of the balls
% 3 separate masks, one for each color
% 1. mask image, blocking out all colors but one 
% 2. find circles
blueImg = createMaskBlue(img);
[Bcenters, Bradii] = imfindcircles(blueImg, [18 55], ... 
    'Sensitivity', 0.85);
% locations = [Bcenters];
sizes = [Bradii(1) * Bradii(1) * pi];
greenImg = createMaskGreen(img); 
[Gcenters, Gradii] = imfindcircles(greenImg, [15 55], ... 
    'Sensitivity', 0.855);
yellowImg = createMaskYellow(img); 
[Ycenters, Yradii] = imfindcircles(yellowImg, [15 55], ... 
    'Sensitivity', 0.855);


imshow('InputImage.png'); 
hold on
% draw circles with different colors
blueCircles = viscircles(Bcenters, Bradii, 'Color', 'c'); 
greenCircles = viscircles(Gcenters, Gradii, 'Color', 'g'); 
yellowCircles = viscircles(Ycenters, Yradii, 'Color', 'y'); 
% mark centers of the circles
plot(Bcenters(:,1), Bcenters(:,2),'b*');
plot(Gcenters(:,1), Gcenters(:,2),'g*');
plot(Ycenters(:,1), Ycenters(:,2),'y*');
disp('Pixels'); 
disp(Bcenters);
% use this to figure out location of the balls in checkerboard reference
% frame
worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), Bcenters);
locations = worldPoints; 
disp('Checker'); 
disp(worldPoints); 
%% Find sizes
GrayImg = rgb2gray(img);
imwrite(GrayImg, 'Gray.png');

sizeMask = segmentImageforSize(GrayImg); 
imwrite(sizeMask, 'forSize.png'); 
% imshow(sizeMask); 
hold on 
[Scenters, Sradii] = imfindcircles(sizeMask, [30 65],'ObjectPolarity','bright', ... 
    'Sensitivity', 0.85,'EdgeThreshold',0.05);
sizeCircles = viscircles(Scenters, Sradii, 'Color', 'r'); 
plot(Scenters(:,1), Scenters(:,2),'r*');


% how many circles we have 
% n = length(Scenters);
% areas = zeros(1, n); 
% for a = 1:n
%     areas(a) = Sradii(a)*Sradii(a)*pi; 
% end
% disp(areas); 


end