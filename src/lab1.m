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
%   SERV_ID = 01;            % we will be talking to server ID 01 on
                % the Nucleo

  SERV_ID = 01;
  DEBUG   = true;          % enables/disables debug prints

  % Instantiate a packet - the following instruction allocates 64
  % bytes for this purpose. Recall that the HID interface supports
  % packet sizes up to 64 bytes.
  packet = zeros(15, 1, 'single');

  % The following code generates a sinusoidal trajectory to be
  % executed on joint 1 of the arm and iteratively sends the list of
  % setpoints to the Nucleo firmware. 
%   viaPts = [0, -400, 400, -400, 400, 0];
    viaPts = [0,0];

  for k = viaPts
      tic
      packet = zeros(15, 1, 'single');
      packet(1) = k;

      % Send packet to the server and get the response      
      %pp.write sends a 15 float packet to the micro controller
       pp.write(SERV_ID, packet); 
       
       pause(0.003); % Minimum amount of time required between write and read
       
       %pp.read reads a returned 15 float Packet from the nucleo.
       returnPacket = pp.read(SERV_ID);
       printMatrix = zeros(1,6);
%        Parse through status packet 
       for x = 0:3
           printMatrix((x*3)+1) = returnPacket((x*3)+1);
           printMatrix((x*3)+2) = returnPacket((x*3)+2);
       end 
       % Output to CSV
        dlmwrite('test.csv', printMatrix, '-append');
       disp(printMatrix);
      toc

      if DEBUG
          disp('Sent Packet:');
          disp(packet);
          disp('Received Packet:');
          disp(returnPacket);
      end
      
      for x = 0:3
          packet((x*3)+1)=0.1;
          packet((x*3)+2)=0;
          packet((x*3)+3)=0;
      end
      
      toc
      pause(1) %timeit(returnPacket) !FIXME why is this needed?
      
  end
  
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Clear up memory upon termination
pp.shutdown()

toc
