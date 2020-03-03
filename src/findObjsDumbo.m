function dumboLocation = findObjsDumbo()
%%   Outputs
%   LOCATION OF BLUE BALL
%% Capture & Undistort the image
cam = webcam();
imOrig = snapshot(cam);
load camParams.mat;
load 'TcamCheck.mat';
load 'T0check.mat';
img = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
imwrite(img, 'InputImage.png');
hold on

%%  Find the colors and locations of the balls
blueImg = createMaskBlue(img);
imshow(blueImg); 
centroids = regionprops( blueImg, 'centroid', ...
    'MajorAxisLength', 'MinorAxisLength');
minAxes = cat(1,centroids.MinorAxisLength);
numCentroids = length(minAxes); 
Bcentroids = cat(1,centroids.Centroid);
axisMax = 0; 
for i = 1:numCentroids
    if (minAxes(i)>axisMax)
        axisMax = minAxes(i);
         dumboLocation = Bcentroids(i, :); 
    end
end 
[objectsDetected, columns] = size(dumboLocation); 
if (objectsDetected == 0)
   dumboLocation =  [-100 0];  
end
imshow('InputImage.png');
hold on
plot(dumboLocation(:,1), dumboLocation(:,2),'b*');
% figure out location of the balls in checkerboard reference frame
worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), dumboLocation);
dumboLocation = worldPoints;
end