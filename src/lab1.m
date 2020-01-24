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
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
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
  PID_SERV_ID = 01;            % we will be talking to server ID 01 on
                % the Nucleo

  STATUS_SERV_ID = 02;
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm 
%   viaPts = [0, -400, 400, -400, 400, 0];
  viaPts = [140 6 -70; 330, 330, 150; -50, 200, 390];
%     viaPts = [0];

  for k = 0:2
      tic
      packet = zeros(15, 1, 'single');
      packet((k*3)+1) = viaPts((k*3)+1);
      pp.write(PID_SERV_ID, packet);
      
      % waits for robot arm and requests status for 1 second per command
      for x = 0:99
          timeStep = 0.01; 
        % send a status request
        packet = zeros(15, 1, 'single');
        pp.write(STATUS_SERV_ID, packet); 
        pause(timeStep); % wait for response
       returnPacket = pp.read(STATUS_SERV_ID);
       % log values into CSV
       printMatrix = zeros(1,4);
       printMatrix(1) = x*timeStep+k; % time stamp for readings.  
       for i = 0:3
           % return packet contains encoder positions
           % convert to angles and put into csv
           printMatrix(i+2) = (returnPacket((i*3)+1)*2*pi)/4095;
                           % this command for enc ticks returnPacket((i*3)+1);
       end 
       % Output to CSV
        dlmwrite('test.csv', printMatrix, '-append');
      end 

%%%%%%%%%%%%%%this stuff is for plotting pretty pictures%%%%%%%%%%%%%%%%%%%

%     hold on  
%     angleLine = animatedline('Color','b', 'LineWidth', 2);
% %     encoderLine = animatedline('Color','r', 'LineWidth', 2);
%     % plots things twice
%     timeInterval = 0.01;
%     for x = 0:1000
%         encPos = returnPacket(1);
%          baseAngle = (encPos*2*pi)/4095; % returns radians 
%           addpoints(angleLine, x*timeInterval, double(baseAngle));
% %           addpoints(encoderLine, x, double(encPos));
%             printMatrix = [x*timeInterval baseAngle encPos];
%             dlmwrite('test.csv', printMatrix, '-append'); 
%          pp.write(STATUS_SERV_ID, packet); 
%         pause(timeInterval); 
%         returnPacket = pp.read(STATUS_SERV_ID);
%     end
%       hold off
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      toc

%       if DEBUG
%           disp('Sent Packet:');
%           disp(packet);
%           disp('Received Packet:');
%           disp(returnPacket);
%       end
%       
%       for x = 0:3
%           packet((x*3)+1)=0.1;
%           packet((x*3)+2)=0;
%           packet((x*3)+3)=0;
%       end
      
      toc
      
  end
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
