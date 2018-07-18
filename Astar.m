function [ angle, distance ] = Astar( start_point, end_point, external_boundaries)
% A* Algorithm
coordinates = AstarHelper(start_point, end_point, external_boundaries);
    x1 = coordinates(1,1);
    y1 = coordinates(1,2);
    x2 = coordinates(2,1);
    y2 = coordinates(2,2);   
        distance = sqrt ( ((y2-y1)^2) + ((x2-x1)^2) );
    angle = atan2d(y2-y1,x2-x1) + 180; 

    if distance > 10
        distance = distance / 2;
    end
    
end

