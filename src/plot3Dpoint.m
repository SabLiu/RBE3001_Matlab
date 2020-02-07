% plots point in 3D space
% takes in XYZ 3x1 matrix
function m = plot3Dpoint(p)
    plot3(p(1),p(2),p(3),'b.');
    hold on, grid on;
end 