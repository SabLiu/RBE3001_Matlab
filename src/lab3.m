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

    
    
    % %% Linear Interpolation : SOMETTHING MIGHT BE WONKY HERE!!!
%     angleConversion = (2*pi)/4095;
%     thetaToEncoder = 4095/(2*pi); 
%     % these are task space positions for each vertex of the triangle
%     % each column is a set of coordinates 
%     vertices = [200 150 170 200; 
%         200 50 -100 200;
%         200 100 150 200];
%     % Linearly interpolate 20 points between each position 
%     P0 = transpose(linearInterpolation(transpose(vertices(:,1)),transpose(vertices(:,2))));
%     P1 = transpose(linearInterpolation(transpose(vertices(:,2)),transpose(vertices(:,3))));
%     P2 = transpose(linearInterpolation(transpose(vertices(:,3)),transpose(vertices(:,4))));
% 
%     % Combine them so the columns are points in task space again 
%     p = [P0 P1 P2];
%     disp(p);
%     result = zeros(1, 7); 
%     for x = 1:60 % Do for each column in p
%         curP = p(:,x); 
%         K = ikin(curP); % returns a 3x1 matrix of theta values
%         
%         % get 3 encoder positions and write to PID server
%         enc1 = K(1)*thetaToEncoder;
%         enc2 = K(2)*thetaToEncoder; 
%         enc3 = K(3)*thetaToEncoder;
%         packet = zeros(15, 1, 'single');
%         packet(1) = enc1;
%         packet(4) = enc2; 
%         packet(7) = enc3; 
%         pp.write(PID_SERV_ID, packet); 
%         
%         printMatrix = zeros(1, 7); 
%         
%         % each setpoint in p is set every 0.1 seconds (2.0s/20 points)
%         % read status every 0.02 seconds: 0.1 s/5
%         for i = 0:4 % do 2(5) times 
%             % Read status 
%             packet = zeros(15, 1, 'single');
%             pp.write(STATUS_SERV_ID, packet); 
%             pause(0.003); 
%             returnPacket = pp.read(STATUS_SERV_ID);
%             pause(0.017); % 0.1/5 = 0.02 seconds per status reading  
%             
%             % prinMatrix : 1x7 row to add to results matrix
%             printMatrix(1) = (.1 * (x-1)) + 0.02*i; % time stamp
%             
%             % Pass in current THETAS to fwkin3001
%             F = fwkin3001(returnPacket(1)*angleConversion, returnPacket(4)*angleConversion, returnPacket(7)*angleConversion); 
%             % these are xyz position in task space 
%             printMatrix(2) = F(1);
%             printMatrix(3) = F(2);
%             printMatrix(4) = F(3);
%             
%             result= [result; printMatrix]; 
% %             plot3Dpoint(F);
%             
%         end 
%         
%         
%     end
%     disp(result);
    
% %% Quintic Trajectory Generation 
%     angleConversion = (2*pi)/4095; % encoder to theta
%     radiansToEncoder = 4095/(2*pi); 
%     % task space positions for each vertex of the triangle
%     % each column is a set of coordinates 
%     vertices = [200 150 170 200; 
%         200 50 -100 200;
%         200 100 150 200];
%     waypoints = zeros(3,1);
%     
%     tf = 2; % each edge takes 2 seconds, 20 waypoints each edge 
%     
%     % for each edge, get coefficient matrices for each joint 
%     % totals 9 matrices 
%     printMatrix = zeros(18,1); 
%     for j = 1:3 
%       L = ikin(vertices(:,j)); 
%       % initial joint encoder positions
%       qo1 = L(1)*radiansToEncoder; 
%       qo2 = L(2)*radiansToEncoder; 
%       qo3 = L(3)*radiansToEncoder;
%         
%       % encoder positions for the next point
%       L = ikin(vertices(:, j+1)); 
%       qf1 = L(1)*radiansToEncoder; 
%       qf2 = L(2)*radiansToEncoder; 
%       qf3 = L(3)*radiansToEncoder; 
%       
%       % pass in start and end encoder positions for each joint on this edge
%       % Get 6x1 coefficient matrix out
%       a = quinticTG(0, tf, qo1, qf1, 0,0,0,0); 
%       b = quinticTG(0, tf, qo2, qf2, 0,0,0,0); 
%       c = quinticTG(0, tf, qo3, qf3, 0,0,0,0); 
%       
%       printMatrix(1:6,1) = a;
%       printMatrix(7:12, 1) = b;
%       printMatrix(13:18,1) = c;
%       dlmwrite('coefficients.csv', printMatrix, '-append');
%       % Get 1x20 matrix of encoder values for that one joint on current
%       % edge
%       enc1 = zeros(1,20);
%       enc2 = zeros(1,20);
%       enc3 = zeros(1,20);
%       
%       enc1 = quinticPositions(a); 
%       enc2 = quinticPositions(b); 
%       enc3 = quinticPositions(c);
%       
%       all3Joints = [enc1; enc2; enc3];
%       
%       % each column in waypoints is encoder positions for a point. 
%       % 3 rows: encoder 1-3
%       if (j==1)
%           waypoints = all3Joints;
%           
%       else
%           waypoints = [waypoints all3Joints];
%       end
%     end
%     disp(waypoints);
%     result = zeros(1, 4); 
%     %   Send columns in waypoints to pidserver and wait 0.1 seconds in between
%     for c = 1:60
%        packet = zeros(15, 1, 'single');
%        packet(1) = waypoints(1,c);
%        packet(4) = waypoints(2,c);
%        packet(7) = waypoints(3,c);
%        
%        pp.write(PID_SERV_ID, packet);
%        pause(0.003);
%        packet = zeros(15, 1, 'single');
%        pp.write(STATUS_SERV_ID, packet);
%        pause(0.003); 
%        returnPacket = pp.read(STATUS_SERV_ID); 
%        pause(0.1 - (2*0.003)); 
% %      printMatrix : 1x4 row to add to results matrix
%         printMatrix = zeros(1,4);
%        printMatrix(1) = (.1 * (c-1)); % time stamp
%        
%        
%        % Pass in current THETAS to fwkin3001
%        F = fwkin3001(returnPacket(1)*angleConversion, returnPacket(4)*angleConversion, returnPacket(7)*angleConversion);
%        % these are xyz position in task space
%        printMatrix(2) = F(1);
%        printMatrix(3) = F(2);
%        printMatrix(4) = F(3);
%        
%        result= [result; printMatrix];
       
%     end


% 
%     %% Graph Position
%     
%     pos1 = result(:,2);
%     pos2 = result(:,3);
%     pos3 = result(:,4);
%     
%     grid on, hold on
%     plot(result(1:61,1), pos1, 'LineWidth', 2.5);
%     plot(result(1:61,1), pos2, 'LineWidth', 2.5);
%     plot(result(1:61,1), pos3, 'LineWidth', 2.5);
% 
%     
%     % Formatting stuff
%     ylim([-150 300]);
%     xlabel('Time (s)'), ylabel('Position (mm)'); 
%     title('End Effector Position (mm) Over Time (s) with Quintic Polynomial Trajectory Generation'); 
%     legend({'End Effector X Position (mm)', 'End Effector Y Position (mm)','End Effector Z Position (mm)'});
%     set(gca, 'fontsize', 16); 
    
    
    
%      % Calculate velocity
%     velocity1 = diff(result(:,2))./0.1;
%     velocity2 = diff(result(:,3))./0.1;
%     velocity3 = diff(result(:,4))./0.1;
%     disp(size(velocity1));
%     
%     grid on, hold on
%     plot(result(1:60,1), velocity1, 'LineWidth', 2.5);
%     plot(result(1:60,1), velocity2, 'LineWidth', 2.5);
%     plot(result(1:60,1), velocity3, 'LineWidth', 2.5);
%     
%     % Formatting stuff
%     ylim([-800 800]);
%     xlabel('Time (s)'), ylabel('Velocity (mm/s)'); 
%     title('End Effector Velocity (mm/s) Over Time (s) with Quintic Polynomial Trajectory Generation'); 
%     legend({'End Effector X Velocity (mm/s)', 'End Effector Y Velocity (mm/s)','End Effector Z Velocity (mm/s)'});
%     set(gca, 'fontsize', 16); 
%     
     %% Calculate acceleration
%     a1 = diff(velocity1) ./ 0.1;
%     a2 = diff(velocity2) ./ 0.1;
%     a3 = diff(velocity3) ./ 0.1;
%     
%     
%     grid on, hold on
%     plot(result(2:60,1), a1, 'LineWidth', 2.5);
%     plot(result(2:60,1), a2, 'LineWidth', 2.5);
%     plot(result(2:60,1), a3, 'LineWidth', 2.5);
%     
%     % Formatting stuff
%     ylim([-50000 50000]);
%     xlabel('Time (s)'), ylabel('Acceleration (mm/s^2 x 10^4)'); 
%     title('End Effector Acceleration (mm/s^2) Over Time (s) with Quintic Polynomial Trajectory Generation'); 
%     legend({'End Effector X Acceleration (mm/s^2)', 'End Effector Y Acceleration (mm/s^2)','End Effector Z Acceleration (mm/s^2)'});
%     set(gca, 'fontsize', 16); 
        
    

    toc
    
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
