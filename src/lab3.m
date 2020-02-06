%% RBE3001 - Laboratory 2

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
%     tic
    %% Status readings
    angleConversion = (2*pi)/4095;
    
    % these are positions for all 3 joints
%     viaPts = [0 -30 -266;0 175 358;0 637 -412;0 -30 -266];
        
        thetaToEncoder = 4095/(2*pi); 
        V = ikin( [200; 200; 200]);
        %position 1: [200; 200; 200]
        % position 2: [100; -150; 150]
        
%         FOR TROUBLESHOOTING ---------------------
%         % encoder values
%         printMatrix(1) = packet(1);
%         printMatrix(2) = packet(4);
%         printMatrix(3) = packet(7);
%         % theta values 
%         printMatrix(4) = V(1);
%         printMatrix(5) = V(2);
%         printMatrix(6) = V(3);
%         % position values expected from fwkin
%         F = fwkin3001(V(1), V(2), V(3)); 
%         printMatrix(7) = F(1);
%         printMatrix(8) = F(2);
%         printMatrix(9) = F(3);
        
         
        
%         for k = 0:14 % do 15 times, total time: 3 seconds for robot to reach setpoint
%             % time step per is 0.2 s 
%             
%             packet = zeros(15, 1, 'single');
%             % Encoder positions
%             packet(1) = V(1)*thetaToEncoder;
%             packet(4) = V(2)*thetaToEncoder;
%             packet(7) = V(3)*thetaToEncoder;
%             % set motor to those positions
%             pp.write(PID_SERV_ID, packet);
% 
%             packet = zeros(15, 1, 'single');
%             pp.write(STATUS_SERV_ID, packet);
%             pause(0.003); % wait for response from server
%             returnPacket = pp.read(STATUS_SERV_ID);
%             
%             % plotting in xyz space
%             %b is P3 returned from plotStickmodel (end effector position)
%             b = plotStickModel([returnPacket(1)*angleConversion, returnPacket(4)*angleConversion, returnPacket(7)*angleConversion]);
%             
%             %printmatrix: time theta1 theta2 theta3 x y z
%             printMatrix = zeros(1, 7);
%             printMatrix(1) = k*0.2; % time steps are 0.2 s
%             
%             % Log joint angles in degrees
%             printMatrix(2) = returnPacket(1)*360/4095;
%             printMatrix(3) = returnPacket(4)*360/4095;
%             printMatrix(4) = returnPacket(7)*360/4095;
%             % Log tip x and z position
%             printMatrix(5) = b(1,1);
%             printMatrix(6) = b(2,1);
%             printMatrix(7) = b(3,1);
%             dlmwrite('ikin.csv', printMatrix, '-append');
    
%             Plot X,Z of robot on a 2D graph
%             xlim([-100 200]);
%             ylim([-30 300]);
%             p = fwkin3001((returnPacket(1)*2*pi)/4095, (returnPacket(4)*2*pi)/4095,(returnPacket(7)*2*pi)/4095);
%             plot(p(1), p(3), 'r*');
%             hold on
%             pause(0.397);
            
%         end
%     plot the 3 setpoints 
%     convert = (2*pi)/4095;
%     p1 = fwkin3001(0*convert, -30*convert, -266*convert);
%     plot(p1(1), p1(3), 'b*');
%     p2 = fwkin3001(0*convert, 175*convert, 358*convert);
%     plot(p2(1), p2(3), 'b*');
%     p3 = fwkin3001( 0*convert, 637*convert, -412*convert);
%     plot(p3(1), p3(3), 'b*');
    
    packet = zeros(15, 1, 'single');
    for x = 0:499
        tic
        pp.write(STATUS_SERV_ID);
        pp.read(STATUS_SERV_ID);
        n = toc;
        dlmwrite('status.csv', n, '-append'); 
    end
    
    
    
%     toc
    hold off
    
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
