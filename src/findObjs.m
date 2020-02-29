function [colors, locations, sizes] = findObjs()
%%   Outputs
%   COLORS found
%   LOCATIONS of balls
%   SIZES of each base (0 is small, 1 is large)

%% Capture & Undistort the image
cam = webcam();
imOrig = snapshot(cam);
load camParams.mat;
load 'TcamCheck.mat';
load 'T0check.mat';
img = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
imwrite(img, 'InputImage.png');

blues = 0;
yellows = 0;
greens = 0;

%%  Find the colors and locations of the balls
% 3 separate masks, one for each color
% 1. mask image, blocking out all colors but one
% 2. find circles
% 3. if found circles of that color, plot on graph
blueImg = createMaskBlue(img);
[Bcenters, Bradii] = imfindcircles(blueImg, [18 55], ...
    'Sensitivity', 0.85);
hold on
if (size(Bcenters)>0)
    blueCircles = viscircles(Bcenters, Bradii, 'Color', 'c');
    plot(Bcenters(:,1), Bcenters(:,2),'b*');
    [blues, m] = size(Bcenters);
end

greenImg = createMaskGreen(img);
[Gcenters, Gradii] = imfindcircles(greenImg, [15 55], ...
    'Sensitivity', 0.855);
if (size(Gcenters)>0)
    greenCircles = viscircles(Gcenters, Gradii, 'Color', 'g');
    plot(Gcenters(:,1), Gcenters(:,2),'g*');
    [greens, m] = size(Gcenters);
end

yellowImg = createMaskYellow(img);
[Ycenters, Yradii] = imfindcircles(yellowImg, [15 55], ...
    'Sensitivity', 0.855);
if (size(Ycenters)>0)
    yellowCircles = viscircles(Ycenters, Yradii, 'Color', 'y');
    plot(Ycenters(:,1), Ycenters(:,2),'y*');
    [yellows,m] = size(Ycenters);
end
allCenters = [Bcenters; Gcenters; Ycenters];


imshow('InputImage.png');
hold on

if ((length(Bradii)==0) && (length(Gradii) == 0)&& (length(Yradii) ==0))
    colors(1) = 5;
    locations = [0 0];
    sizes(1) = 5;
else
    % figure out location of the balls in checkerboard reference frame
    worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), allCenters);
    locations = worldPoints;
    
    %% Find sizes
    GrayImg = rgb2gray(img);
    imwrite(GrayImg, 'Gray.png');
    
    sizeMask = segmentImageforSize(GrayImg);
    imwrite(sizeMask, 'forSize.png');
    hold on
    [Scenters, Sradii] = imfindcircles(sizeMask, [30 70],'ObjectPolarity','bright', ...
        'Sensitivity', 0.85,'EdgeThreshold',0.05);
    sizeCircles = viscircles(Scenters, Sradii, 'Color', 'r');
    % how many circles we have
    [n, columns] = size(Scenters);
    sizes = zeros(1, n);
    % add sizes of circles in
    disp('find objects N: ');
    disp(n);
    disp('length Sradii');
    disp(length(Sradii));
    if (length(Sradii)>0)
        plot(Scenters(:,1), Scenters(:,2),'r*');
        for a = 1:n
            area=  Sradii(a)*Sradii(a)*pi;
            if (area > 8000)
                sizes(a) = 1;
            else
                sizes(a) = 0;
            end
        end
    elseif (length(Bradii)+length(Gradii)+length(Yradii) ~= length(Sradii))
            colors(1) = 5; 
    end
    
    % add in colors
    colors = zeros(1,n);
    if (blues > 0)
        for b = 1:blues
            colors(b) = 1;
        end
    end
    if (greens > 0)
        for g = 1:greens
            colors(blues+g) = 2;
        end
    end
    if (yellows>0)
        for y = 1:yellows
            colors(blues + greens + y) = 3;
        end
    end
end
end