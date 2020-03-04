function marieLocation = findObjsMarie()
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
% blueImg = createMaskBlue(img);
blueImg = createMaskMarie(img);
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
         marieLocation = Bcentroids(i, :); 
    end
end 
[objectsDetected, columns] = size(marieLocation); 
if (objectsDetected == 0)
   marieLocation =  [-100 0];  
end
imshow('InputImage.png');
hold on
plot(marieLocation(:,1), marieLocation(:,2),'b*');
% figure out location of the balls in checkerboard reference frame
worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), marieLocation);
marieLocation = worldPoints;
end