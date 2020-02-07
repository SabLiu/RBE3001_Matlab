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
    tic
%% Graph 
    angleConversion = (2*pi)/4095;
    thetaToEncoder = 4095/(2*pi); 
    % these are task space positions for each vertex of the triangle
    % each column is a set of coordinates 
    vertices = [200 150 170 200; 
        200 50 -100 200;
        200 100 150 200];
    % Linearly interpolate 20 points between each position 
    P0 = transpose(linearInterpolation(transpose(vertices(:,1)),transpose(vertices(:,2))));
    P1 = transpose(linearInterpolation(transpose(vertices(:,2)),transpose(vertices(:,3))));
    P2 = transpose(linearInterpolation(transpose(vertices(:,3)),transpose(vertices(:,4))));

    % Combine them so the columns are points in task space again 
    p = [P0 P1 P2];
    disp(p);
    for x = 1:60 % Do for each column in p
        curP = p(:,x); 
        K = ikin(curP); % returns a 3x1 matrix of theta values
        
        % get 3 encoder positions and write to PID server
        enc1 = K(1)*thetaToEncoder;
        enc2 = K(2)*thetaToEncoder; 
        enc3 = K(3)*thetaToEncoder;
        packet = zeros(15, 1, 'single');
        packet(1) = enc1;
        packet(4) = enc2; 
        packet(7) = enc3; 
        pp.write(PID_SERV_ID, packet); 
        
        printMatrix = zeros(1, 4); 
        
        % each setpoint in p is set every 0.1 seconds (2.0s/20 points)
        % read status every 0.02 seconds: 0.1 s/5
        for i = 0:4 % do 5 times 
            % Read status 
            packet = zeros(15, 1, 'single');
            pp.write(STATUS_SERV_ID, packet); 
            pause(0.003); 
            returnPacket = pp.read(STATUS_SERV_ID);
            pause(0.017); % 0.1/5 = 0.02 seconds per status reading  
            
            printMatrix(1) = (.1 * (x-1)) + 0.02*i; % time stamp
            
            % Pass in current THETAS to fwkin3001
            F = fwkin3001(returnPacket(1)*angleConversion, returnPacket(4)*angleConversion, returnPacket(7)*angleConversion); 
            % these are xyz position in task space 
            printMatrix(2) = F(1);
            printMatrix(3) = F(2);
            printMatrix(4) = F(3);
            plot3Dpoint(F);
            dlmwrite('3dTriangleInterpolation.csv', printMatrix, '-append');
        end 
        
    end 
        
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
    
%     packet = zeros(15, 1, 'single');
%     for x = 0:4
%         pp.write(STATUS_SERV_ID,packet);
%         pause(0.003);
%         
%         returnPacket = pp.read(STATUS_SERV_ID);
%         printMatrix = zeros(1, 4); 
%         printMatrix(1) = returnPacket(1); 
%         printMatrix(2) = returnPacket(4); 
%         printMatrix(3) = returnPacket(7); 
%         
%         dlmwrite('status.csv', printMatrix, '-append'); 
%         pause(4); 
%     end
    toc
    hold off
    
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
