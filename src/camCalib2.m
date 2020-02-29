% Auto-generated by cameraCalibrator app on 19-Feb-2020
%-------------------------------------------------------


% Define images to process
imageFileNames = {'/ifs/home/stliu/RBE3001_Matlab06/src/Image1.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image4.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image5.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image6.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image7.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image8.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image9.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image11.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image12.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image13.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image14.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image15.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image16.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image17.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image18.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image19.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image21.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image22.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image23.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image24.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image25.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image26.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image27.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image28.png',...
    '/ifs/home/stliu/RBE3001_Matlab06/src/Image29.png',...
    };

% Detect checkerboards in images
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames);
imageFileNames = imageFileNames(imagesUsed);

% Read the first image to obtain image size
originalImage = imread(imageFileNames{1});
[mrows, ncols, ~] = size(originalImage);

% Generate world coordinates of the corners of the squares
squareSize = 12;  % in units of 'millimeters'
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', false, ...
    'NumRadialDistortionCoefficients', 2, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);

% View reprojection errors
h1=figure; showReprojectionErrors(cameraParams);

% Visualize pattern locations
h2=figure; showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% For example, you can use the calibration data to remove effects of lens distortion.
undistortedImage = undistortImage(originalImage, cameraParams);

% See additional examples of how to use the calibration data.  At the prompt type:
% showdemo('MeasuringPlanarObjectsExample')
% showdemo('StructureFromMotionExample')
