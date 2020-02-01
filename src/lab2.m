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
    %% Status readings
    timeStep = 0.5;
    angleConversion = (2*pi)/4095;
    
    % these are positions for all 3 joints
%     viaPts = [0 -30 -266;0 175 358;0 637 -412;0 -30 -266];
    viaPts = [0	-30	-266;
        0	-26.4820288	-255.2916352;
        0	-16.6965504	-225.5055616;
        0	-1.7962176	-180.1503104;
        0	17.0663168	-122.7344128;
        0	38.7384	-56.7664;
        0	62.0673792	14.2451968;
        0	85.9006016	86.7918464;
        0	109.0854144	157.3650176;
        0	130.4691648	222.4561792;
        0	148.8992	278.5568;
        0	163.2228672	322.1583488;
        0	172.2875136	349.7522944;
        0	174.9404864	357.8301056;
        
        0	175	358;
        0	182.9283392	344.78624;
        0	204.9815936	308.03136;
        0	238.5621184	252.06496;
        0	281.0722688	181.21664;
        0	329.9144	99.816;
        0	382.4908672	12.19264;
        0	436.2040256	-77.32384;
        0	488.4562304	-164.40384;
        0	536.6498368	-244.71776;
        0	578.1872	-313.936;
        0	610.4706752	-367.72896;
        0	630.9026176	-401.76704;
        0	636.8853824	-411.72064;
        
        0	637	-412;
        0	625.5538496	-409.4945792;
        0	593.7155968	-402.5255936;
        0	545.2355392	-391.9139584;
        0	483.8639744	-378.4805888;
        0	413.3512	-363.0464;
        0	337.4475136	-346.4323072;
        0	259.9032128	-329.4592256;
        0	184.4685952	-312.9480704;
        0	114.8939584	-297.7197568;
        0	54.9296	-284.5952;
        0	8.3258176	-274.3953152;
        0	-21.1670912	-267.9410176;
        0	-29.7988288	-266.0532224
        ];

    for k = 0:41 % do this 42 times because 42 viapoints
        packet = zeros(15, 1, 'single');
        
        % will need to set positions with these for viaPts
        packet(1) = viaPts(k+1, 1); % this is for joint 0
        packet(4) = viaPts(k+1, 2); % this is for joint 1
        packet(7) = viaPts(k+1, 3); % this is for joint 2
        % set motor to those positions
        pp.write(PID_SERV_ID, packet);

%         for x = 0:50
            packet = zeros(15, 1, 'single');
            pp.write(STATUS_SERV_ID, packet);
            pause(0.003);
            returnPacket = pp.read(STATUS_SERV_ID);
            
            % plotting in xyz space
            %b is P3 returned from plotStickmodel (end effector position)
%             b = plotStickModel([returnPacket(1)*angleConversion, returnPacket(4)*angleConversion, returnPacket(7)*angleConversion]);
            %printmatrix: time theta1 theta2 theta3 x z
%             printMatrix = zeros(1,12);
%             printMatrix(1) = x*.103 +(k * 50 * .103);%this gross number is the .1 timestep plus the .003 seconds to receive return packet
%             printMatrix(2) = returnPacket(1)*360/4095;
%             printMatrix(3) = returnPacket(4)*360/4095;
%             printMatrix(4) = returnPacket(7)*360/4095;
%             printMatrix(5) = b(1,1);
%             printMatrix(6) = b(3,1);
%             printMatrix(7) = returnPacket(1);
%             printMatrix(8) = returnPacket(4);
%             printMatrix(9) = returnPacket(7);
%             dlmwrite('triangle.csv', printMatrix, '-append');
    
            % Plot X,Z of robot on a 2D graph
            xlim([-100 200]);
            ylim([-30 300]);
            p = fwkin3001((returnPacket(1)*2*pi)/4095, (returnPacket(4)*2*pi)/4095,(returnPacket(7)*2*pi)/4095);
            plot(p(1), p(3), 'r*');
            hold on
            pause(0.397);
            
            
%         end
    end    
    % plot the 3 setpoints 
    convert = (2*pi)/4095;
    p1 = fwkin3001(0*convert, -30*convert, -266*convert);
    plot(p1(1), p1(3), 'b*');
    p2 = fwkin3001(0*convert, 175*convert, 358*convert);
    plot(p2(1), p2(3), 'b*');
    p3 = fwkin3001( 0*convert, 637*convert, -412*convert);
    plot(p3(1), p3(3), 'b*');
     
    toc
    hold off
    
    
    
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
T1 = tdh(-theta1, L1, 0, -pi/2);
T2 = tdh(-theta2, 0, L2, 0);
T3 = tdh(-theta3 +(pi/2) + pi/2,0,L3,0);
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

% q : 3x1 with joint angles (thetas)
function B = plotStickModel(q)
% Static link lengths for 3001 robot
L1 = 135;
L2 = 175;
L3 = 169.28;
% Base
P0 = [0; 0; 0];
% Transformation matrices
T1 = tdh(-q(1), L1, 0, -pi/2);
T2 = T1*tdh(-q(2), 0, L2, 0);
T3 = T2*tdh(pi/2 - q(3), 0, L3, 0);
% Position vectors from the T matrices
P1 = T1(1:3, 4);
% disp('P1');
% disp(P1);
P2 = T2(1:3, 4);
% disp('P2');
% disp(P2);
P3 = T3(1:3, 4);
% disp('P3');
% disp(P3);
% Create a matrix of all the points
allPoints = [P0 P1 P2 P3];
% Create arrays for X,Y,Z to plot
X = allPoints(1,:);
Y = allPoints(2,:);
Z = allPoints(3,:);

% Create 3D plot
% Plot links
robotLinks = plot3(X,Y,Z);
hold on
% Plot joint points
robotPoints = plot3(X,Y,Z,'r.');
grid on
axis([0 300 -300 300 -50 300]);
% h = triad('matrix', T1);
%     H = get(h, 'matrix');
%     for theta = 0:360
%         set(h,'matrix',T1,'xrotate', deg2rad(theta), 'yrotate', deg2rad(theta), 'zrotate', deg2rad(theta));
%         drawnow
%     end
%      h=   triad('matrix',T1);
pause(0.397);
delete(robotPoints);
delete(robotLinks);
B = P3;
end
function h = triad(varargin)
%% Find or default parent
idx = find( strcmpi('parent',varargin) );
if ~isempty(idx)
    if numel(idx) > 1
        idx = idx(end);
        warning(sprintf('Multiple Parents are specified, using %d.',idx));
    end
    mom = varargin{idx+1};
    axs = ancestor(mom,'axes','toplevel');
    hold(axs,'on');
else
    mom = gca;
    hold(mom,'on');
end
%% Create triad
h = hgtransform('Parent',mom);
kids(1) = plot3([0,1],[0,0],[0,0],'Color',[1,0,0],'Tag','X-Axis','Parent',h);
kids(2) = plot3([0,0],[0,1],[0,0],'Color',[0,1,0],'Tag','Y-Axis','Parent',h);
kids(3) = plot3([0,0],[0,0],[0,1],'Color',[0,0,1],'Tag','Z-Axis','Parent',h);
%% Update properties
for i = 1:2:numel(varargin)
    switch lower(varargin{i})
        case 'linestyle'
            set(kids,varargin{i},varargin{i+1});
        case 'linewidth'
            set(kids,varargin{i},varargin{i+1});
        case 'matrix'
            set(h,varargin{i},varargin{i+1});
        case 'parent'
            %do nothing, property handled earlier
            %set(h,varargin{i},varargin{i+1});
            %daspect([1 1 1]);
        case 'scale'
            s = varargin{i+1};
            if numel(s) == 1
                s = repmat(s,1,3);
            end
            if numel(s) ~= 3
                error('The scaling factor must be a singular value or a 3-element array.');
            end
            for j = 1:numel(kids)
                xdata = get(kids(j),'XData');
                ydata = get(kids(j),'YData');
                zdata = get(kids(j),'ZData');
                set(kids(j),'XData',xdata*s(1),'YData',ydata*s(2),'ZData',zdata*s(3));
            end
        case 'tag'
            set(h,varargin{i},varargin{i+1});
        otherwise
            % TODO - add check for properties in line or hgtransform, and
            % update property accordingly.
            warning(sprintf('Ignoring "%s," unexpected property.',varargin{i}));
    end
end
end