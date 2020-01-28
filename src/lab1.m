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
    
    %% Motor Control
    % these are 3 arbitrary positions for all 3 joints
    viaPts = [-471.25 -24.25 1222.5; 
        -242 640.25 253.3; 
        327.5 28.25 188.8; 
        137.75 350.25 -347.45];
    
    for k = 0:3
        tic
        packet = zeros(15, 1, 'single');
        
        % will need to set positions with these for viaPts
        packet(1) = viaPts(k+1, 1); % this is for joint 0
        packet(4) = viaPts(k+1, 2); % this is for joint 1
        packet(7) = viaPts(k+1, 3); % this is for joint 2
        
        % set motor to those positions
        pp.write(PID_SERV_ID, packet);
        disp(k);
        
        hold on
        encoderLine1 = animatedline('Color','r', 'LineWidth', 2);
        encoderLine2 = animatedline('Color','g', 'LineWidth', 2);
        encoderLine3 = animatedline('Color','b', 'LineWidth', 2);
        
        xlim([0 3]);
        ylim([-500 1500]);
        %% waits for robot arm and requests status
        for x = 0:500
            timeStep = 0.01;
            
            % send a status request
            packet = zeros(15, 1, 'single');
            pp.write(STATUS_SERV_ID, packet);
            pause(timeStep); 
            returnPacket = pp.read(STATUS_SERV_ID);
            
            % read encoder position from return packet
            encPos1 = returnPacket(1); 
            encPos2 = returnPacket(4); 
            encPos3 = returnPacket(7); 
            
            % plot on graph
            addpoints(encoderLine1, x*timeStep, double(encPos1));
            addpoints(encoderLine2, x*timeStep, double(encPos2));
            addpoints(encoderLine3, x*timeStep, double(encPos3));
            
            % log values into CSV
            printMatrix = zeros(1,4);
            printMatrix(1) = x*timeStep+k; % time stamp for readings.
            for i = 0:3
                printMatrix(i+2) = returnPacket((i*3)+1);
                    % for angles: (returnPacket((i*3)+1)*2*pi)/4095;
            end
            dlmwrite('test.csv', printMatrix, '-append');
        end
        hold off
        toc
        
    end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
