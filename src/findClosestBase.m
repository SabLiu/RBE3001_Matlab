% Order the colors array so they align with ordering of bases
% find which base is closest to the center
% put center in that index

function [orderedColors, orderedBallCenters] = findClosestBase(bases, colorCenters, colors)
[n, cols] = size(bases); 
orderedColors = zeros(1, n);
% coordinates
orderedBallCenters = zeros(n, 2); 
% for each base size (iterate down rows)
for i = 1:n
    minXdiff = 1000;
    % current base X coordinate
    curX = bases(i,1);
    % find closest color coordinate
    for b = 1:n
%         disp(b);
        colorX = colorCenters(b,1);
        curDiff = abs(colorX - curX);
        if (minXdiff > curDiff)
            minXdiff = curDiff;
            minXindex = b;
        end
    end
   % move ordered colors and centers
   orderedColors(i) = colors(minXindex); 
   orderedBallCenters(i, 1) = colorCenters(minXindex, 1);
   orderedBallCenters(i, 2) = colorCenters(minXindex, 2);
end
end