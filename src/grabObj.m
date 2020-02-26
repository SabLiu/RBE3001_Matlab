% function grabObj()
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
grid on
    %% Setup
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
        packet = zeros(15, 1, 'single');
    packet(1) = 1; 
    pp.write(GRIPPER_SERV, packet); 
    pause(.003);
% camRobotRegistration(); 
load 'TcamCheck.mat'
load 'T0check.mat'
% [T0_check, T_cam_to_checker] = camRobotRegistration();   
[colors, locations, sizes] = findObjs();
disp(locations); 

angleToEncoder = 4095/(2*pi);
encoderToAngle = (2*pi)/4095;

% points in robot's reference frame
TworldPoints = transpose(locations); 
roboPoints = T0_check* [TworldPoints;    0; 1];
disp(roboPoints); 

% read current joint positions 
packet = zeros(15, 1, 'single');
% pp.write(STATUS_SERV_ID, packet); 
pause(0.003); 
% returnPacket = pp.read(STATUS_SERV_ID); 
% q10 = returnPacket(1)*encoderToAngle; 
% q20 = returnPacket(4)*encoderToAngle; 
% q30 = returnPacket(7)*encoderToAngle; 

% ikin to get the joint variables
% pass in 1 point for now
roboPoints(3) = 50; 
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

% coefficients
C1 = generateTraj(0, 1, 0, 0, qInitial(1), q(1)); 
C2 = generateTraj(0, 1, 0, 0, qInitial(2), q(2)); 
C3 = generateTraj(0, 1, 0, 0, qInitial(3), q(3)); 

enc1 = solveCubic(C1); 
enc2 = solveCubic(C2);
enc3 = solveCubic(C3);

all3Joints = [enc1; enc2; enc3]; 
% move to neighborhood of ball 
for c = 1:20
    packet = zeros(15, 1, 'single');
    packet(1) = all3Joints(1,c)*angleToEncoder;
    packet(4) = all3Joints(2,c)*angleToEncoder;
    packet(7) = all3Joints(3,c)*angleToEncoder;
    pp.write(PID_SERV_ID, packet); 
    pause(0.05); 
end
% move down to ball
% want to go to X,Y, lower Z
% get current X, Y and supply Z 
% then find the joint angles we need to get there (qfinal)

packet = zeros(15, 1, 'single');
pp.write(STATUS_SERV_ID, packet);
pause(0.003); 
returnPacket = pp.read(STATUS_SERV_ID);

FWK = fwkin3001(returnPacket(1)*encoderToAngle,returnPacket(4)*encoderToAngle,returnPacket(7)*encoderToAngle); 
X = FWK(1,4); 
Y = FWK(2, 4); 
% find current X and Y positions and supply Z
if (Y>0)
    extraY = 8; 
else
    extraY = -8
end
    Yoffset = roboPoints(2) - Y;
    disp(Yoffset); 
    Xoffset = roboPoints(1) - X;
    disp(Xoffset); 
anglesFinal = ikin([X+Xoffset; Y+Yoffset+extraY; -10]); 
qInitial = [returnPacket(1)*encoderToAngle;
    returnPacket(4)*encoderToAngle;
    returnPacket(7)*encoderToAngle];
C1 = generateTraj(0, 1, 0, 0, qInitial(1), anglesFinal(1)); 
C2 = generateTraj(0, 1, 0, 0, qInitial(2), anglesFinal(2)); 
C3 = generateTraj(0, 1, 0, 0, qInitial(3), anglesFinal(3)); 

enc1 = solveCubic(C1); 
enc2 = solveCubic(C2);
enc3 = solveCubic(C3);

all3Joints = [enc1; enc2; enc3]; 
% move down, slowly and smoothly
for c = 1:20
    packet = zeros(15, 1, 'single');
    packet(1) = all3Joints(1,c)*angleToEncoder;
    packet(4) = all3Joints(2,c)*angleToEncoder;
    packet(7) = all3Joints(3,c)*angleToEncoder;
    pp.write(PID_SERV_ID, packet); 
    pause(0.05); 
end

% disp(fwkin3001(q1, q2, q3)); 

packet = zeros(15, 1, 'single');
pp.write(STATUS_SERV_ID, packet);
pause(0.003); 
returnPacket = pp.read(STATUS_SERV_ID);
disp('end pos');
disp(fwkin3001(returnPacket(1)*encoderToAngle,returnPacket(4)*encoderToAngle,returnPacket(7)*encoderToAngle)); 
        packet = zeros(15, 1, 'single');
    packet(1) = 0; 
    pp.write(GRIPPER_SERV, packet); 
    pause(3);
    packet(1) = 1; 
    pp.write(GRIPPER_SERV, packet); 
    pause(3);
pp.shutdown();
% end 
toc
