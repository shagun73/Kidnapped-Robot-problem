function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify the map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);

scans=30;
numParticles = 400;
botSim.setScanConfig(botSim.generateScanConfig(scans));

%% Particle Filter
maxNumOfIterations = 30;

%virtualBot is the estimated botSim
[botSim, virtualBot] = PFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);    

%% Path Planning

%% Plot the chosen path
end_point = target;

LocationRobot = virtualBot.getBotPos();
while LocationRobot(1) > end_point(1) + 0.002 || LocationRobot(1) < end_point(1) - 0.002 || LocationRobot(2) > end_point(2) + 0.002 || LocationRobot(2) < end_point(2) - 0.002;
    
    [angle, distance] = Astar(LocationRobot, end_point, map);
    Robot_direction = virtualBot.getBotAng();
     angleRadian = (angle) * pi / 180; 
    if botSim.debug()
        botSim.drawBot(5,'red');
        virtualBot.drawBot(5,'green');
    end 
    botSim.turn(pi-Robot_direction+angleRadian);
    virtualBot.turn(pi-Robot_direction+angleRadian);   
    botScan = botSim.ultraScan();
    if botScan(1)<= distance;
        [botSim, virtualBot] = PFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);
    else
        botSim.move(distance);
        virtualBot.move(distance);
    end   
    botScan = botSim.ultraScan();
    virtualBotScan = virtualBot.ultraScan();
    difference = (sum(virtualBotScan-botScan)/scans);
    threshold = 3;
    if (abs(difference) > threshold)
        [botSim, virtualBot] = PFilter(botSim, modifiedMap,numParticles, maxNumOfIterations, scans);
    end
    LocationRobot = virtualBot.getBotPos();
    if botSim.debug()
        pause(1);
    end
end
% Drawing

if botSim.debug()
    figure(1)
    hold off;
    botSim.drawMap(); 
    botSim.drawBot(30,'g');
    virtualBot.drawBot(30,'r'); 
    drawnow;
end

end
