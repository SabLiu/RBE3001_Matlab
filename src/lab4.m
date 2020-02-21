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
 
%     radiansToEncoder = 4095/(2*pi);
%     % These points send the robot to a singularity. 
%     vertices = [200 L2+L3 200 100;
%                 200 0 200 100;
%                 200 L1 100 200]; 
% 
%     % These points are our "legal" triangle. 
% % %     vertices = [200 150 170 200;
% % %         200 50 -100 200;
% % %         200 100 150 200];
%     thetas = zeros(3,1);
%     % generate traj: interpolate in task space
%     % want each edge to take 2 seconds 
%     coefficients = zeros(4,9);
%     
%     % For each edge
%     for curEdge = 1:4
%         curPoint = vertices(:,curEdge);
%         nextPoint = vertices(:, curEdge+1); 
%         % Generate coefficients for x, y, or z for that edge
%         for xyz = 1:3
%             column = (curEdge-1)*3 + xyz; 
%             % edge 1  edge 2  edge 3
%             % x y z | x y z| x y z|
%             coefficients(:, column) = generateTraj(0, 2, 0, 0, curPoint(xyz), nextPoint(xyz)); 
%         end
%     end
%     
%     %generates the task space position matrix
%     taskSpacePos = zeros(3, 60);
%     
%     %for each edge
%     for eachEdge = 1:3
%         %for each time stamp
%         %each timestamp will be 0.1 seconds (2 secconds/20 timestamps)
%         for time = 0:19
%             %for each xyz row
%             for xyz = 1:3
%                 %pulls out a coeffecinet from each column
%                 column = (eachEdge - 1)*3 + xyz;
%                 curCoeff = coefficients(:,column);
%                 %pulls out a coeffectinet from each row
%                 %solves for solve cubic
%                 column = (eachEdge-1)*20 + (time + 1);
%                 taskSpacePos(xyz,column) = solveCubic(curCoeff, 0.1*time);
%             end
%         end
%     end
%     
%     statusThetas = zeros(3,60);
%     statusEncoders = zeros(3,60); 
%     
%     %For each Task Space Point from column 1-60
%     for points = 1:60
%         
%         %pulls out current point from Task Space Position and runs it
%         %through our inverse kinematic equations
%         curPoint = taskSpacePos(:,points);
%         thetas = ikin(curPoint);
%         %stores values into 3x1 matirx 'encoders'
%         %Multiplies each value by 'radiansToEncoder' before transfer
%         encoders = zeros(3,1);
%         for theta = 1:3
%             encoders(theta) = thetas(theta)*radiansToEncoder;
%         end
%         
%         packet = zeros(15, 1, 'single');
%         packet(1) = encoders(1);
%         packet(4) = encoders(2);
%         packet(7) = encoders(3);
%         
%         % Send position to motors to execute
%         pp.write(PID_SERV_ID, packet);
%         pause(0.003);
%         packet = zeros(15, 1, 'single');
%         pp.write(STATUS_SERV_ID, packet);
%         pause(0.003); 
%         returnPacket = pp.read(STATUS_SERV_ID);
%         thetas(1) = returnPacket(1)*angleConversion;
%         thetas(2) = returnPacket(4)*angleConversion;
%         thetas(3) = returnPacket(7)*angleConversion;
%         
%         statusThetas(:, points) = thetas;
%         statusEncoders = [returnPacket(1); returnPacket(4); returnPacket(7)];
%         
%         % Calculate instantaneous velocity for x,y, and z
%         if (points ~= 1)robotPoints =
%             q = statusThetas(:, points); 
%             singularityEStop(q); 
%             
%             % Calculations for instantaneous velocity vector
%             dq = (q - statusThetas(:, points-1))./0.1; 
%             % dp is instantaneous task space velocity
%             dp = fwdVel(q, dq); 
%             endEffectorPos = fwkin3001(q(1), q(2), q(3)); 
%             
%             % Calculate J for manip. ellipsoid
%             J = jacob0(q);
%             Jp = J(1:3,1:3);
%             A = Jp*transpose(Jp);
%             ellipsoid = plot_ellipse(A, [endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4)]); 
% %             velocityVector = quiver3(endEffectorPos(1,4), endEffectorPos(2,4), endEffectorPos(3,4), dp(1), -dp(2), dp(3)); 
%             plotStickModel(thetas);
%             
%             eigA = sqrt(eig(A));
%             volume = (4/3 * pi) * eigA(1)*eigA(2)*eigA(3);
%             disp(volume);
%             
%             % Clear the plot: delete ellipsoid, robot, velocity vector
%             delete(ellipsoid); 
%             
%         end
%         
%     end

%% EXTRA CREDIT
for i = 0:2
    packet = zeros(15, 1, 'single');
    pp.write(STATUS_SERV_ID, packet);
    pause(0.003);
    returnPacket = pp.read(STATUS_SERV_ID);
    
    encoderToRadians = (2*pi)/4095; % encoder to theta
    radiansToEncoder = 4095/(2*pi);
    
    %% Determine current position
    q = [returnPacket(1)*encoderToRadians; returnPacket(4)*encoderToRadians; returnPacket(7)*encoderToRadians];
    fwkin = fwkin3001(returnPacket(1)*encoderToRadians, returnPacket(2)*encoderToRadians, returnPacket(7)*encoderToRadians);
    p0 = fwkin(1:3, 4);
    plot2DStickModel(q);
    
    %% Determine target point
    % User manually selects target point on graph
    targetPoint = ginput(1);
    disp(targetPoint);
    
    pd = [targetPoint(1); 0; targetPoint(2)];
    solution = invKinAlg(q, pd);
    disp(solution);
    
    packet = zeros(15, 1, 'single');
    packet(1) = solution(1)*radiansToEncoder;
    packet(4) = solution(2)*radiansToEncoder;
    packet(7) = solution(3)*radiansToEncoder;
    pp.write(PID_SERV_ID, packet);
    
    disp(packet);
    for i = 0:20
        packet = zeros(15, 1, 'single');
        pp.write(STATUS_SERV_ID, packet);
        pause(0.003);
        returnPacket = pp.read(STATUS_SERV_ID);
        q = [returnPacket(1)*encoderToRadians; returnPacket(4)*encoderToRadians; returnPacket(7)*encoderToRadians];
        plot2DStickModel(q);
        pause(0.1);
    end
end
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
