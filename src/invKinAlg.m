
% pe is desired position (3x1)

function jointAngles = invKinAlg(pe)
%% Setup
    encoderToRadians = (2*pi)/4095; % encoder to theta
    radiansToEncoder = 4095/(2*pi);
 % Plot stick model of robot at arbitrary configuration in xz plane
    q0 = [0;pi/4; pi/6];
    plot2DStickModel(q0);
    
    targetPoint = ginput(1); 
    disp(targetPoint);
    
    pd = [targetPoint(1); 0; targetPoint(2)]; 
    plot2DStickModel(ikin(pd)); 
    
    %% Move to target
%     q = [returnPacket(1)*encoderToRadians; returnPacket(4)*encoderToRadians; returnPacket(7)*encoderToRadians]; 
%     p0 = fwkin3001(returnPacket(1)*encoderToRadians, returnPacket(2)*encoderToRadians, returnPacket(7)*encoderToRadians);
%     deltaQ = pe - p0;
    
    % while the robot is still 50 mm away from its desired position
%     while (deltaQ > 50)
%         % !!! need to define the invJacobian
%         deltaQ = invJacobian*(pe - jacob0(q)); 
%         
%         q = [returnPacket(1)*encoderToRadians; returnPacket(4)*encoderToRadians; returnPacket(7)*encoderToRadians]; 
%         p0 = fwkin3001(returnPacket(1)*encoderToRadians, returnPacket(2)*encoderToRadians, returnPacket(7)*encoderToRadians);
%         deltaQ = pe - p0; 
%         
%     end
end