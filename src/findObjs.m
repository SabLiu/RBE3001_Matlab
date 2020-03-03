function [colors, locations, sizes] = findObjs()
%%   Outputs
%   COLORS found
%   LOCATIONS of balls wrt Checkerboard RF
%   SIZES of each base (0 is small, 1 is large)

%% Capture & Undistort the image
cam = webcam();
imOrig = snapshot(cam);
load camParams.mat;
load 'TcamCheck.mat';
load 'T0check.mat';
img = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
imwrite(img, 'InputImage.png');
hold on
blues = 0;
yellows = 0;
greens = 0;

%%  Find the colors and locations of the balls
% 3 separate masks, one for each color
% 1. mask image, blocking out all colors but one
% 2. find circles
% 3. if found circles of that color, plot on graph
blueImg = createMaskBlue(img);
[Bcenters, Bradii] = imfindcircles(blueImg, [20 55], ...
    'Sensitivity', 0.85);
[blues, m] = size(Bcenters);

greenImg = createMaskGreen(img);
[Gcenters, Gradii] = imfindcircles(greenImg, [20 55], ...
    'Sensitivity', 0.855);
[greens, m] = size(Gcenters);

yellowImg = createMaskYellow(img);
[Ycenters, Yradii] = imfindcircles(yellowImg, [20 55], ...
    'Sensitivity', 0.855);
[yellows,m] = size(Ycenters);

allCenters = [Bcenters; Gcenters; Ycenters];
% disp('initial allCenters');
% disp(allCenters); 

imshow('InputImage.png');
hold on

if ((length(Bradii)==0) && (length(Gradii) == 0)&& (length(Yradii) ==0))
    colors(1) = 5;
    locations = [0 0];
    sizes(1) = 5;
else
    %% Find sizes
    GrayImg = rgb2gray(img);
    imwrite(GrayImg, 'Gray.png');
    
    sizeMask = segmentImageforSize(GrayImg);
    imwrite(sizeMask, 'forSize.pnframeg');
    [Scenters, Sradii] = imfindcircles(sizeMask, [30 70],'ObjectPolarity','bright', ...
        'Sensitivity', 0.85,'EdgeThreshold',0.05);
    sizeCircles = viscircles(Scenters, Sradii, 'Color', 'r');
    % how many circles we have
    [n, columns] = size(Scenters);
    sizes = zeros(1, n);
    
    
    % add in colors
    colors = zeros(1,n);
    if (blues > 0)
        for b = 1:blues
            colors(b) = 1;
        end
        blueCircles = viscircles(Bcenters, Bradii, 'Color', 'c');
        plot(Bcenters(:,1), Bcenters(:,2),'b*');
        
    end
    if (greens > 0)
        for g = 1:greens
            colors(blues+g) = 2;
        end
        greenCircles = viscircles(Gcenters, Gradii, 'Color', 'g');
        plot(Gcenters(:,1), Gcenters(:,2),'g*');
    end
    if (yellows>0)
        for y = 1:yellows
            colors(blues + greens + y) = 3;
        end
        yellowCircles = viscircles(Ycenters, Yradii, 'Color', 'y');
        plot(Ycenters(:,1), Ycenters(:,2),'y*');
    end
%         disp('old Colors');
%     disp(colors);
%     disp('old centers');
%     disp(allCenters);
    [newColors, newAllCenters] = findClosestBase(Scenters, allCenters, colors);
    colors = newColors;
    allCenters = newAllCenters;
        % figure out location of the balls in checkerboard reference frame
    worldPoints = pointsToWorld(cameraParams, T_cam_to_checker(1:3,1:3), T_cam_to_checker(1:3,4), allCenters);
    locations = worldPoints;
%     disp('new Colors');
%     disp(colors);
%     disp('new centers');
%     disp(allCenters);
% 
%     disp('find objects N: ');
%     disp(n);
    if (length(Sradii)>0)
        % make sure that the size circle corresponds to the right cente
        
        % plot base circles and find areas
        plot(Scenters(:,1), Scenters(:,2),'r*');
        for a = 1:n
            area=  Sradii(a)*Sradii(a)*pi;
%             disp(area);
            if (area > 7000)
                sizes(a) = 1;
            else
                sizes(a) = 0;
            end
        end
    elseif ((n ~= length(Sradii))||(n==0 && length(Sradii)==0))
        colors(1) = 5;
    end
    
end
end