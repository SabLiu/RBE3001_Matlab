%%
% RBE3001 - Laboratory 1
%
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
%

clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

disp (vid);
disp (pid);

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

% Static link lengths for 3001 robot
    L1 = 135; 
    L2 = 175;
    L3 = 169.28;
    grid on
try
    %% Setup
    % Define all server IDs
    PID_SERV_ID = 01;
    PID_CONFIG_SERV_ID = 02;
    STATUS_SERV_ID = 10;
    DEBUG   = true;          % enables/disables debug prints
    
    packet = zeros(15, 1, 'single');
    
    % Set PID values
    % Joint 0 
    packet(1) = 0.0015;
    packet(2) = 0.0000095;
    packet(3) = 0.000001;
    % Joint 1
    packet(4) = 0.0008;
      packet(5) = 0.005;
    % Joint 2
    packet(7) = 0.004;
    packet(8) = 0.0007;
    packet(9) = 0.00013;
    
    pp.write(PID_CONFIG_SERV_ID,packet);
    pause(0.003);
    
    timeStep = .01;
    tic
    %% Status readings
%     for x = 0:9
%         % send a status request
%         packet = zeros(15, 1, 'single');
%         pp.write(STATUS_SERV_ID, packet);
%         pause(timeStep);
%         returnPacket = pp.read(STATUS_SERV_ID);
%         
%         printMatrix = zeros(1,4);
%         printMatrix(1) = x*timeStep; % time stamp for readings.
%         for i = 0:2
%             printMatrix(i+2) = returnPacket((i*3)+1);
%         end
%         dlmwrite('test.csv', printMatrix, '-append');
%     end
    m = [0;0;0];
    plotStickModel(m);
    
    hold off
        toc
     

    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc

 function p = fwkin3001(theta1, theta2, theta3)
 % Static link lengths for 3001 robot
    L1 = 135; 
    L2 = 175;
    L3 = 169.28;
    % Find transformation matrices
    T1 = tdh(theta1,L1, 0, -pi/2);
    T2 = tdh(theta2, 0, L2, 0);
    T3 = tdh(theta3+(pi/2),0,L3,0);
    % Calculate transformation matrix from 0 to 3
    T = T1*T2*T3; 
    % Only return the 3x1 position matrix
    p = T(1:3, 4);
 end

 function A = tdh(theta, d, a, alpha)
    A = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
         sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0 sin(alpha) cos(alpha) d;
         0 0 0 1];
 end
 
 function B = plotStickModel(q)
    % Static link lengths for 3001 robot
    L1 = 135; 
    L2 = 175;
    L3 = 169.28;
    
    P0 = [0; 0; 0];
    T1 = tdh(q(1), L1, 0, -pi/2);
    T2 = T1*tdh(q(2), 0, L2, 0);
    T3 = T2*tdh(pi/2 + q(3), 0, L3, 0);
    P1 = T1(1:3, 4);
    P2 = T2(1:3, 4);
    P3 = T3(1:3, 4);

    
    allPoints = [P0 P1 P2 P3];
    X = allPoints(1,:);
    Y = allPoints(2,:);
    Z = allPoints(3,:);
   
    plot3(X,Y,Z);
    grid on
    axis([0 300 -300 300 -50 300]);
     h=   triad('matrix',T1);
    H = get(h, 'Matrix');
    for theta = 0:360
          set(h,'Matrix',T1);
          drawnow
      end
     %         triad('matrix',T2);
     
 end