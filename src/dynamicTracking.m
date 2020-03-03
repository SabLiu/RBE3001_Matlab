% function grabObj()
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
% run first time to find objects
Bcenters = findObjsDT();
% disp(Bcenters);
[numObjects, ignore] = size(Bcenters);

% find points of objects in robot's reference frame
TworldPoints = transpose(Bcenters);
% need to make sure matrix multiplication works
% add 0s and 1s according to how many objects are detected
addBottom = zeros(2, numObjects);
for col = 1:numObjects
    addBottom(2,col) = 1;
end
roboPoints = T0_check* [TworldPoints;  addBottom];

timer = 0;
%% MAIN LOOP
% colors(1) will be 5 if there are no more circles detected.
% colors is returned by findObjs()
while (timer<10)
    
    % ikin to get the joint variables
    roboPoints(3,1) = 100;
    q = ikin(roboPoints(1:3, 1));
    prevPoints = roboPoints(1:3,1);
    packet = zeros(15, 1, 'single');
    packet(1) = q(1)*angleToEncoder;
    packet(4) = q(2)*angleToEncoder;
    packet(7) = q(3)*angleToEncoder;
    pp.write(PID_SERV_ID, packet);
    pause(0.003);
    % take new snapshot of workspace and find new object position
    Bcenter = findObjsDT();
    if (Bcenter(1) <-50)
        roboPoints = prevPoints;
    else
        % find points of next object in robot's reference frame
        TworldPoints = transpose(Bcenter);
        addBottom = zeros(2, numObjects);
        for col = 1:numObjects
            addBottom(2,col) = 1;
        end
        roboPoints = T0_check* [TworldPoints;  addBottom];
    end
    timer = timer+1;
end

pp.shutdown();


toc
