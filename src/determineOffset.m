% takes in X and Y position 
% determines the extra X and Y offsets needed
function [Xoffset, Yoffset] = determineOffset(x,y)

if ((y>-75)&&(y<75))
   Xoffset = 10; 
   Yoffset = -30; 
%    disp('Quadrant 1,8,9');
elseif((x<190)&&(y>=75))
    Xoffset = 28; 
    Yoffset = -35; 
%     disp('Quadrant 2,6');
elseif((x>190)&&(y>=75))
    Xoffset = 28;
    Yoffset = -55;
%     disp('Quadrant 5');
elseif((x<140)&&(y<-75))
    Xoffset = 5; 
    Yoffset = -10; 
%     disp('Quadrant 7');
elseif((x<190)&&(x>140)&&(y<-75))
    Xoffset = 5;
    Yoffset = -15;
%     disp('Quadrant 3');
    
elseif ((x>=190)&&(y<-75))
    Xoffset = 0; 
    Yoffset = -35;
%     disp('Quadrant 4');
else
    Xoffset = 0; 
    Yoffset = 0;
%     disp('Quadrant ?');
end
% disp('Xoffset');
% disp(Xoffset);
% disp('Yoffset');
% disp(Yoffset); 
end