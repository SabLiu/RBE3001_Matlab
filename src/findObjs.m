function [imDetectedDisk, robotFramePose, diskDia] = findObjs(imOrig, T_checker_to_robot, T_cam_to_checker, cameraParams)
% FINDOBJS implements a sequence of image processing steps to detect
% any objects of interest that may be present in an RGB image.
%
%   Usage
%   -----
%   [IMDETECTEDOBJS, ROBOTFRAMEPOSE] = findObjs(IMORIG, TCHECKER2ROBOT, TCAM2CHECKER, CAMERAPARAMS)
%
%   Inputs
%   ------
%   IMORIG - an RGB image showing the robot's workspace (capture from a CAM
%   object).
%
%   TCHECKER2ROBOT - the homogeneous transformation matrix between the
%   checkered board and the reference frame at the base of the robot.
%
%   TCAM2CHECKER - the homogeneous transformation matrix between the camera
%   reference frame and the checkered board (you can calculate this using
%   the GETCAMTOCHECKERBOARD function, provided separately).
%
%   CAMERAPARAMS - an object containing the camera's intrinsic and
%   extrinsic parameters, as returned by MATLAB's camera calibration app.
%
%%   Outputs
%   IMDETECTEDOBJS - a binarized image showing the location of the
%   segmented objects of interest.
%   
%   ROBOTFRAMEPOSE - the coordinates of the objects expressed in the robot's
%   reference frame
% see https://www.mathworks.com/help/vision/ref/cameraparameters.pointstoworld.html
% for details on the expected dimensions for YOUR_PIXEL_VALUES)


%%  Undistort the image using the camera parameters
% [im, ~] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
img = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
RGB = imread('InputImage.png'); 

%% THIS WORKS, DO NOT TOUCH. STEP 5&6A
%%  Find the colors and locations of the balls
blueImg = createMaskBlue(img); 
[Bcenters, Bradii] = imfindcircles(blueImg, [20 55], ... 
    'Sensitivity', 0.87);
greenImg = createMaskGreen(img); 
[Gcenters, Gradii] = imfindcircles(greenImg, [20 55], ... 
    'Sensitivity', 0.87);
yellowImg = createMaskYellow(img); 
[Ycenters, Yradii] = imfindcircles(yellowImg, [20 55], ... 
    'Sensitivity', 0.87);


imshow('InputImage.png'); 
hold on
blueCircles = viscircles(Bcenters, Bradii, 'Color', 'c'); 
greenCircles = viscircles(Gcenters, Gradii, 'Color', 'g'); 
yellowCircles = viscircles(Ycenters, Yradii, 'Color', 'y'); 
% use this to figure out location of the balls
% worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), centers);
plot(Bcenters(:,1), Bcenters(:,2),'b*');
plot(Gcenters(:,1), Gcenters(:,2),'g*');
plot(Ycenters(:,1), Ycenters(:,2),'y*');


%% Find sizes
GrayImg = rgb2gray(RGB);
imwrite(GrayImg, 'Gray.png');

sizeMask = segmentImageforSize(GrayImg); 
imwrite(sizeMask, 'forSize.png'); 
% imshow('forSize.png'); 
hold on 
[Scenters, Sradii] = imfindcircles(sizeMask, [30 60],'ObjectPolarity','bright', ... 
    'Sensitivity', 0.89,'EdgeThreshold',0.05);
sizeCircles = viscircles(Scenters, Sradii, 'Color', 'r'); 
plot(Scenters(:,1), Scenters(:,2),'r*');
% how many circles we have 
n = length(Scenters);
areas = zeros(1, n); 
for a = 1:n
    areas(a) = Sradii(a)*Sradii(a)*pi; 
end
disp(areas); 

% BWimgSize = createMaskSize(img); % outputs binarized image
% % centers: xy pixel coordinates 
% [centers,radii] = imfindcircles(BWimgSize,[50 100], ...
%     'Sensitivity',0.87);
% imshow(BWimgSize); 
% hold on
% h = viscircles(centers, radii);

%% Find the size
% h = fspecial('average'); 
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);
% img = imfilter(img, h);imshow(img); 
% BWimgSize = createMaskSize(img);
% [centers,radii] = imfindcircles(BWimgSize,[50 150], ...
%     'Sensitivity',0.9);
% BWimgSize = edge(BWimgSize, 'approxcanny');
% fill = imfill(BWimgSize, 'holes'); 
% 
% imshow(BWimgSize); 
% hold on; 
% imshow(fill);
% h = viscircles(centers,radii); 



end