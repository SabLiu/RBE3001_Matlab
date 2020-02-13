%% RBE3001 - Laboratory 4

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
    
    
    %% Live Plot of Task-Space Velocity Vector
    angleConversion = (2*pi)/4095; % encoder to theta
    radiansToEncoder = 4095/(2*pi);
    vertices = [200 150 170 200;
        200 50 -100 200;
        200 100 150 200];
    thetas = zeros(3,1);
%     for v = 1:4
%         packet = zeros(15, 1, 'single');
%         curPoints = vertices(:,v); 
%         packet(1) = curPoints(1);
%         packet(4) = curPoints(2);
%         packet(7) = curPoints(3); 
%         pp.write(PID_SERV_ID,packet);
%         pause(0.003);
%     status = zeros(3,20); 
%         for i = 1:10
%             packet = zeros(15, 1, 'single');
%             pp.write(STATUS_SERV_ID,packet);
%             pause(0.003);
%             returnPacket = pp.read(STATUS_SERV_ID);
%             thetas(1) = returnPacket(1)*angleConversion;
%             thetas(2) = returnPacket(4)*angleConversion;
%             thetas(3) = returnPacket(7)*angleConversion;
%             plotStickModel(thetas);
%             status(:, i+1) = thetas; 
%             
%             pause(2);
%             if (i ~= 1)
%                 q = status(:, i);
%                 dq = (q - status(:, i-1))./0.2;
%                 % dp is instantaneous task space velocity
%                 dp = fwdVel(q, dq);
%                 endEffectorPos = fwkin3001(q(1), q(2), q(3));
%                 
%                 velocityVector = quiver3(endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4), dp(1), dp(2), dp(3));
%                 plotStickModel(thetas);
%                 delete(velocityVector);
%             end
%             
%         end
% %         pause(2);
%     end
    % generate traj: interpolate in task space
    % want each edge to take 2 seconds 
    coefficients = zeros(4,9);
    
    % For each edge
    for curEdge = 1:3
        curPoint = vertices(:,curEdge);
        nextPoint = vertices(:, curEdge+1); 
        % Generate coefficients for x, y, or z for that edge
        for xyz = 1:3
            column = (curEdge-1)*3 + xyz; 
            % edge 1  edge 2  edge 3
            % x y z | x y z| x y z|
            coefficients(:, column) = generateTraj(0, 2, 0, 0, curPoint(xyz), nextPoint(xyz)); 
        end
    end
    
    %generates the task space position matrix
    taskSpacePos = zeros(3, 60);
    
    %for each edge
    for eachEdge = 1:3
        %for each time stamp
        %each timestamp will be 0.1 seconds (2 secconds/20 timestamps)
        for time = 0:19
            %for each xyz row
            for xyz = 1:3
                %pulls out a coeffecinet from each column
                column = (eachEdge - 1)*3 + xyz;
                curCoeff = coefficients(:,column);
                %pulls out a coeffectinet from each row
                %solves for solve cubic
                column = (eachEdge-1)*20 + (time + 1);
                taskSpacePos(xyz,column) = solveCubic(curCoeff, 0.1*time);
            end
        end
    end
    
    statusThetas = zeros(3,60);
    statusEncoders = zeros(3,60); 
    
    %For each Task Space Point from column 1-60
    for points = 1:60
        
        %pulls out current point from Task Space Position and runs it
        %through our inverse kinematic equations
        curPoint = taskSpacePos(:,points);
        thetas = ikin(curPoint);
        %stores values into 3x1 matirx 'encoders'
        %Multiplies each value by 'radiansToEncoder' before transfer
        encoders = zeros(3,1);
        for theta = 1:3
            encoders(theta) = thetas(theta)*radiansToEncoder;
        end
        
        packet = zeros(15, 1, 'single');
        packet(1) = encoders(1);
        packet(4) = encoders(2);
        packet(7) = encoders(3);
        
        pp.write(PID_SERV_ID, packet);
        pause(0.003);
        packet = zeros(15, 1, 'single');
        pp.write(STATUS_SERV_ID, packet);
        pause(0.003); 
        returnPacket = pp.read(STATUS_SERV_ID);
        thetas(1) = returnPacket(1)*angleConversion;
        thetas(2) = returnPacket(4)*angleConversion;
        thetas(3) = returnPacket(7)*angleConversion;
        
        statusThetas(:, points) = thetas;
        statusEncoders = [returnPacket(1); returnPacket(4); returnPacket(7)];
        
        % Calculate instantaneous velocity for x,y, and z
        if (points ~= 1)
            q = statusThetas(:, points); 
            dq = (q - statusThetas(:, points-1))./0.1; 
            % dp is instantaneous task space velocity
            dp = fwdVel(q, dq); 
            endEffectorPos = fwkin3001(q(1), q(2), q(3)); 
            
            velocityVector = quiver3(endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4), dp(1), -dp(2), dp(3)); 
            plotStickModel(thetas);
            delete(velocityVector);
        end
        
    end
    
    
%     %% Quintic Trajectory Generation
% 
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
%         L = ikin(vertices(:,j));
%         % initial joint encoder positions
%         qo1 = L(1)*radiansToEncoder;
%         qo2 = L(2)*radiansToEncoder;
%         qo3 = L(3)*radiansToEncoder;
%         
%         % encoder positions for the next point
%         L = ikin(vertices(:, j+1));
%         qf1 = L(1)*radiansToEncoder;
%         qf2 = L(2)*radiansToEncoder;
%         qf3 = L(3)*radiansToEncoder;
%         
%         % pass in start and end encoder positions for each joint on this edge
%         % Get 6x1 coefficient matrix out
%         a = quinticTG(0, tf, qo1, qf1, 0,0,0,0);
%         b = quinticTG(0, tf, qo2, qf2, 0,0,0,0);
%         c = quinticTG(0, tf, qo3, qf3, 0,0,0,0);
%         
%         printMatrix(1:6,1) = a;
%         printMatrix(7:12, 1) = b;
%         printMatrix(13:18,1) = c;
%         dlmwrite('coefficients.csv', printMatrix, '-append');
%         % Get 1x20 matrix of encoder values for that one joint on current
%         % edge
%         enc1 = zeros(1,20);
%         enc2 = zeros(1,20);
%         enc3 = zeros(1,20);
%         
%         enc1 = quinticPositions(a);
%         enc2 = quinticPositions(b);
%         enc3 = quinticPositions(c);
%         
%         all3Joints = [enc1; enc2; enc3];
%         
%         % each column in waypoints is encoder positions for a point.
%         % 3 rows: encoder 1-3
%         if (j==1)
%             waypoints = all3Joints;
%             
%         else
%             waypoints = [waypoints all3Joints];
%         end
%     end
%     disp(waypoints);
%     result = zeros(1, 4);
%     %   Send columns in waypoints to pidserver and wait 0.1 seconds in between
%     for c = 1:60
%         packet = zeros(15, 1, 'single');
%         packet(1) = waypoints(1,c);
%         packet(4) = waypoints(2,c);
%         packet(7) = waypoints(3,c);
%         
%         pp.write(PID_SERV_ID, packet);
%         pause(0.003);
%         packet = zeros(15, 1, 'single');
%         pp.write(STATUS_SERV_ID, packet);
%         pause(0.003);
%         returnPacket = pp.read(STATUS_SERV_ID);
%         pause(0.1 - (2*0.003));
%         %      printMatrix : 1x4 row to add to results matrix
%         printMatrix = zeros(1,4);
%         printMatrix(1) = (.1 * (c-1)); % time stamp
%         
%         
%         % Pass in current THETAS to fwkin3001
%         F = fwkin3001(returnPacket(1)*angleConversion, returnPacket(4)*angleConversion, returnPacket(7)*angleConversion);
%         % these are xyz position in task space
%         printMatrix(2) = F(1);
%         printMatrix(3) = F(2);
%         printMatrix(4) = F(3);
%         
%         result= [result; printMatrix];
%         plot3Dpoint(F);
%         
%     end
%     
%     
    
    toc
    
    
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc

% run fwkin with three target thetas
% fwkin returns the p matrix 0 to i 
% pass the p matrices into jacobian

% q = [0; pi/2; pi/2];
% j = jacob0(q);
% disp(j);
% disp(det(j(1:3, 1:3)));