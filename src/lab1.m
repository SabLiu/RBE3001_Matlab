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
    % Joint 0 (works!)
    packet(1) = 0.0015;
    packet(2) = 0.0000095;
    packet(3) = 0.000001;
    % Joint 1
    packet(4) = 0.2;
    %   packet(5) = 0.00000001;
    %   packet(6) = 0.00000001;
    % Joint 2
    packet(7) = 0.0044;
    packet(8) = 0.0002;
    packet(9) = 0.00009;
    
    pp.write(PID_CONFIG_SERV_ID,packet);
    pause(0.003);
    
    %% Motor Control
    % these are 3 arbitrary positions for all 3 joints
    viaPts = [300];%[140 6 -70; 330, 330, 150; -50, 200, 390];
    
    for k = 0   %0:1
        tic
        packet = zeros(15, 1, 'single');
        packet(1) = -300;
        packet(4) = 100;
        packet(7) = 200;
        
        % will need to set positions with these for viaPts
%         packet((k*3)+1) = viaPts((k*3)+1); % this is for joint 0
%         packet((k*3)+4) = viaPts((k*3)+1); % this is for joint 0
%         packet((k*3)+7) = viaPts((k*3)+1); % this is for joint 0
        
        % set motor to those positions
        pp.write(PID_SERV_ID, packet);
        
        hold on
        encoderLine = animatedline('Color','r', 'LineWidth', 2);
        xlim([0 1]);
        ylim([0,210]);
        %% waits for robot arm and requests status
        for x = 0:100
            timeStep = 0.01;
            
            % send a status request
            packet = zeros(15, 1, 'single');
            pp.write(STATUS_SERV_ID, packet);
            pause(timeStep); 
            returnPacket = pp.read(STATUS_SERV_ID);
            
            % read encoder position from return packet
            encPos = returnPacket(7); 
            
            % plot on graph
            addpoints(encoderLine, x*timeStep, double(encPos));
            
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
