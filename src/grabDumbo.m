
%% Initialize
clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);
tic

% Define all server IDs

PID_SERV_ID = 01;
PID_CONFIG_SERV_ID = 02;
STATUS_SERV_ID = 10;
GRIPPER_SERV = 55;

packet = zeros(15, 1, 'single');

% Set PID values
% Joint 0
packet(1) = 0.0015;
packet(2) = 0.0000095;
packet(3) = 0.000001;
% Joint 1ts1(t+1) = solveCubic(C1, 0.3 * time);
packet(4) = 0.0008;
packet(5) = 0.005;
% Joint 2
packet(7) = 0.004;
packet(8) = 0.0007;
packet(9) = 0.00013;

pp.write(PID_CONFIG_SERV_ID,packet);
pause(0.003);
angleToEncoder = 4095/(2*pi);
encoderToAngle = (2*pi)/4095;

% open gripper
packet = zeros(15, 1, 'single');
packet(1) = 1;
pp.write(GRIPPER_SERV, packet);
pause(.003);
load camParams.mat;
load 'TcamCheck.mat'
load 'T0check.mat'

%% Find DUMBO

    packet = zeros(15, 1, 'single');dumboLocation = findObjsDumbo();
[numObjects,m] = size(dumboLocation);
% find points of objects in robot's reference frame
TworldPoints = transpose(dumboLocation);
% need to make sure matrix multiplication works
% add 0s and 1s according to how many objects are detected
addBottom = zeros(2, numObjects);
for col = 1:numObjects
    addBottom(2,col) = 1;
end
roboPoints = T0_check* [TworldPoints;  addBottom];

% these variables are used for trajectory generation.
timeSteps = 50;
timeInts = 0.05;
totalTime = 2.5;
    % ikin to get the joint variables
    roboPoints(3,1) = 50;
    q = ikin(roboPoints(1:3, 1));
    %% Generate trajectory
    % get current position
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    qInitial = [returnPacket(1)*encoderToAngle;
        returnPacket(4)*encoderToAngle;
        returnPacket(7)*encoderToAngle];
    %% move to neighborhood of ball
    % generate 6x1 matrices of coefficients
    C1 = generateTraj(0, totalTime, 0, 0, qInitial(1), q(1));
    C2 = generateTraj(0, totalTime, 0, 0, qInitial(2), q(2));
    C3 = generateTraj(0, totalTime, 0, 0, qInitial(3), q(3));
    % find timeStepsx1 matrices of encoder positions
    enc1 = solveCubic(C1);
    enc2 = solveCubic(C2);
    enc3 = solveCubic(C3);
    % concatenate all 3 encoder paths
    all3Joints = [enc1; enc2; enc3];
    % iterate through each column in all3Joints and send to PID server
    for c = 1:timeSteps
        packet = zeros(15, 1, 'single');
        packet(1) = all3Joints(1,c)*angleToEncoder;
        packet(4) = all3Joints(2,c)*angleToEncoder;
        packet(7) = all3Joints(3,c)*angleToEncoder;
        pp.write(PID_SERV_ID, packet);
        pause(timeInts);
    end
    %% move down to ball
    % want to go to X,Y, lower Z
    % get current X, Y and supply Z
    % then find the joint angles we need to get there (qfinal)
    
    % current XY position
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    FWK = fwkin3001(returnPacket(1)*encoderToAngle,returnPacket(4)*encoderToAngle,returnPacket(7)*encoderToAngle);
    X = FWK(1,4);
    Y = FWK(2, 4);
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    % factor in residual error
    Yoffset = roboPoints(2) - Y;
    Xoffset = roboPoints(1) - X;
    [extraX, extraY] = determineOffset(X, Y);
    anglesFinal = ikin([X+Xoffset+extraX; Y+Yoffset+extraY; -30]);
    qInitial = [returnPacket(1)*encoderToAngle;
        returnPacket(4)*encoderToAngle;
        returnPacket(7)*encoderToAngle];
    % find coefficients for traj. gen.
    C1 = generateTraj(0, totalTime, 0, 0, qInitial(1), anglesFinal(1));
    C2 = generateTraj(0, totalTime, 0, 0, qInitial(2), anglesFinal(2));
    C3 = generateTraj(0, totalTime, 0, 0, qInitial(3), anglesFinal(3));
    enc1 = solveCubic(C1);
    enc2 = solveCubic(C2);
    enc3 = solveCubic(C3);
    all3Joints = [enc1; enc2; enc3];
    % move down, slowly and smoothly
    for c = 1:timeSteps
        packet = zeros(15, 1, 'single');
        packet(1) = all3Joints(1,c)*angleToEncoder;
        packet(4) = all3Joints(2,c)*angleToEncoder;
        packet(7) = all3Joints(3,c)*angleToEncoder;
        pp.write(PID_SERV_ID, packet);
        pause(timeInts);
    end
    
    % Grab Dumbo
    disp('I got Dumbo!');
    packet = zeros(15, 15, 'single');
    packet(1) = 0;
    pp.write(GRIPPER_SERV, packet);
    pause(0.5);
    
    % move to correct dropoff location
    desiredX = 100; 
    desiredY = 200;
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    
    FWK = fwkin3001(returnPacket(1)*encoderToAngle,returnPacket(4)*encoderToAngle,returnPacket(7)*encoderToAngle);
    X = FWK(1,4);
    Y = FWK(2, 4);
    anglesFinal = ikin([desiredX; desiredY; 150]);
    qInitial = [returnPacket(1)*encoderToAngle;
        returnPacket(4)*encoderToAngle;
        returnPacket(7)*encoderToAngle];
    C1 = generateTraj(0, totalTime, 0, 0, qInitial(1), anglesFinal(1));
    C2 = generateTraj(0, totalTime, 0, 0, qInitial(2), anglesFinal(2));
    C3 = generateTraj(0, totalTime, 0, 0, qInitial(3), anglesFinal(3));
    enc1 = solveCubic(C1);
    enc2 = solveCubic(C2);
    enc3 = solveCubic(C3);
    all3Joints = [enc1; enc2; enc3];
    % move to location, slowly and smoothly
    for c = 1:timeSteps
        packet = zeros(15, 1, 'single');
        packet(1) = all3Joints(1,c)*angleToEncoder;
        packet(4) = all3Joints(2,c)*angleToEncoder;
        packet(7) = all3Joints(3,c)*angleToEncoder;
        pp.write(PID_SERV_ID, packet);
        pause(timeInts);
    end
    % release gripper
    packet = zeros(15, 1, 'single');
    packet(1) = 1;
    pp.write(GRIPPER_SERV, packet);
    pause(0.5);
    
    
    %% Now, find Marie!
    marieLocation = findObjsMarie();
[numObjects,m] = size(marieLocation);
% find points of objects in robot's reference frame
TworldPoints = transpose(marieLocation);
% need to make sure matrix multiplication works
% add 0s and 1s according to how many objects are detected
addBottom = zeros(2, numObjects);
for col = 1:numObjects
    addBottom(2,col) = 1;
end
roboPoints = T0_check* [TworldPoints;  addBottom];

    roboPoints(3,1) = 50;
    q = ikin(roboPoints(1:3, 1));
    % Generate trajectory
    % get current position
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    qInitial = [returnPacket(1)*encoderToAngle;
        returnPacket(4)*encoderToAngle;
        returnPacket(7)*encoderToAngle];
    % move to neighborhood of ball
    % generate 6x1 matrices of coefficients
    C1 = generateTraj(0, totalTime, 0, 0, qInitial(1), q(1));
    C2 = generateTraj(0, totalTime, 0, 0, qInitial(2), q(2));
    C3 = generateTraj(0, totalTime, 0, 0, qInitial(3), q(3));
    % find timeStepsx1 matrices of encoder positions
    enc1 = solveCubic(C1);
    enc2 = solveCubic(C2);
    enc3 = solveCubic(C3);
    % concatenate all 3 encoder paths
    all3Joints = [enc1; enc2; enc3];
    % iterate through each column in all3Joints and send to PID server
    for c = 1:timeSteps
        packet = zeros(15, 1, 'single');
        packet(1) = all3Joints(1,c)*angleToEncoder;
        packet(4) = all3Joints(2,c)*angleToEncoder;
        packet(7) = all3Joints(3,c)*angleToEncoder;
        pp.write(PID_SERV_ID, packet);
        pause(timeInts);
    end
    % move down to marie
    % current XY position
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    FWK = fwkin3001(returnPacket(1)*encoderToAngle,returnPacket(4)*encoderToAngle,returnPacket(7)*encoderToAngle);
    X = FWK(1,4);
    Y = FWK(2, 4);
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    % factor in residual error
    Yoffset = roboPoints(2) - Y;
    Xoffset = roboPoints(1) - X;
    [extraX, extraY] = determineOffset(X, Y);
    anglesFinal = ikin([X+Xoffset+extraX; Y+Yoffset+extraY; -30]);
    qInitial = [returnPacket(1)*encoderToAngle;
        returnPacket(4)*encoderToAngle;
        returnPacket(7)*encoderToAngle];
    % find coefficients for traj. gen.
    C1 = generateTraj(0, totalTime, 0, 0, qInitial(1), anglesFinal(1));
    C2 = generateTraj(0, totalTime, 0, 0, qInitial(2), anglesFinal(2));
    C3 = generateTraj(0, totalTime, 0, 0, qInitial(3), anglesFinal(3));
    enc1 = solveCubic(C1);
    enc2 = solveCubic(C2);
    enc3 = solveCubic(C3);
    all3Joints = [enc1; enc2; enc3];
    % move down, slowly and smoothly
    for c = 1:timeSteps
        packet = zeros(15, 1, 'single');
        packet(1) = all3Joints(1,c)*angleToEncoder;
        packet(4) = all3Joints(2,c)*angleToEncoder;
        packet(7) = all3Joints(3,c)*angleToEncoder;
        pp.write(PID_SERV_ID, packet);
        pause(timeInts);
    end
    
    % Grab marie
    disp('I got Marie!');
    packet = zeros(15, 15, 'single');
    packet(1) = 0;
    pp.write(GRIPPER_SERV, packet);
    pause(0.5);
    
    % move to correct dropoff location
    desiredX = 100; 
    desiredY = -200;
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    
    FWK = fwkin3001(returnPacket(1)*encoderToAngle,returnPacket(4)*encoderToAngle,returnPacket(7)*encoderToAngle);
    X = FWK(1,4);
    Y = FWK(2, 4);
    anglesFinal = ikin([desiredX; desiredY; 150]);
    qInitial = [returnPacket(1)*encoderToAngle;
        returnPacket(4)*encoderToAngle;
        returnPacket(7)*encoderToAngle];
    C1 = generateTraj(0, totalTime, 0, 0, qInitial(1), anglesFinal(1));
    C2 = generateTraj(0, totalTime, 0, 0, qInitial(2), anglesFinal(2));
    C3 = generateTraj(0, totalTime, 0, 0, qInitial(3), anglesFinal(3));
    enc1 = solveCubic(C1);
    enc2 = solveCubic(C2);
    enc3 = solveCubic(C3);
    all3Joints = [enc1; enc2; enc3];
    % move to location, slowly and smoothly
    for c = 1:timeSteps
        packet = zeros(15, 1, 'single');
        packet(1) = all3Joints(1,c)*angleToEncoder;
        packet(4) = all3Joints(2,c)*angleToEncoder;
        packet(7) = all3Joints(3,c)*angleToEncoder;
        pp.write(PID_SERV_ID, packet);
        pause(timeInts);
    end
    % release gripper
    packet = zeros(15, 1, 'single');
    packet(1) = 1;
    pp.write(GRIPPER_SERV, packet);
    pause(0.5);
    

pp.shutdown();


toc
