
function [PID_SERV_ID STATUS_SERV_ID GRIPPER_SERV] = initialize()

clear
% clear java
% clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');
% 
% disp (vid);
% disp (pid);

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

grid on
    %% Setup
    % Define all server IDs
    
    PID_SERV_ID = 01;
    PID_CONFIG_SERV_ID = 02;
    STATUS_SERV_ID = 10;
    GRIPPER_SERV = 55; 
    
    serverParams = [PID_SERV_ID STATUS_SERV_ID GRIPPER_SERV]; 
    
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
%     
%     packet = zeros(15, 1, 'single');
%     packet(1) = 1; 
%     pp.write(GRIPPER_SERV, packet); 
%     pause(.003);
    

end