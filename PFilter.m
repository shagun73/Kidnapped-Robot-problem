function [botSim, virtualBot] = PFilter(botSim, modifiedMap,ParticleCount, maxParticleCountOfIterations, scans)
%Particle Filter Localisation Function

particles(ParticleCount,1) = BotSim; 
for i = 1:ParticleCount
    particles(i) = BotSim(modifiedMap);  
    particles(i).randomPose(10); 
    particles(i).setScanConfig(generateScanConfig(particles(i), scans));
end

n = 0;
while(n < maxParticleCountOfIterations) 
    n = n+1;  
    botScan = botSim.ultraScan(); %get a scan from the real robot.
    
    var = 70;   %variance
    %% Write code for updating your particles scans
    weight = zeros(ParticleCount,1);
    pWeight = zeros(scans,1);
    pScan = zeros(scans,ParticleCount);
    diff = zeros(scans,ParticleCount);
    
    for i=1:ParticleCount
        
        if particles(i).insideMap() == 0
            particles(i).randomPose(0);
        end
        
        pScan(:,i)= particles(i).ultraScan();
        for j=1:scans
            %% Write code for scoring your particles
            p = circshift(pScan(:,i),j); 
            diff(j,i) = sqrt(sum((p-botScan).^2)); 
            pWeight(j) = (1/sqrt(2*pi*var))*exp(-((diff(j,i))^2/(2*var)));
        end
        [max_weight, max_pos] = max(pWeight);
        direction_angle = particles(i).getBotAng() + max_pos*2*pi/scans;
        weight(i) = max_weight;
        particles(i).setBotAng(mod(direction_angle, 2*pi));
    end
        
    weights = weight./sum(weight);
    
    %% Write code for resampling your particles
    
    Freshlocation = zeros(ParticleCount, 3);
    
    for i = 1:ParticleCount
        j = find(rand() <= cumsum(weights),1);
        Freshlocation(i, 1:2) = particles(j).getBotPos();
        Freshlocation(i, 3) = particles(j).getBotAng();
    end  
    for i=1:ParticleCount
        particles(i).setBotPos([Freshlocation(i,1), Freshlocation(i,2)]);
        particles(i).setBotAng(Freshlocation(i,3));
    end
    % Estimating particle position     
    angles = zeros(ParticleCount,1);
    positions = zeros(ParticleCount, 2);
    
    for i = 1:ParticleCount
        positions(i,:) = particles(i).getBotPos();
        angles(i)=particles(i).getBotAng();
    end
    
    vMode = BotSim(modifiedMap);
    vMode.setScanConfig(vMode.generateScanConfig(scans));
    vMode.setBotPos(mode(positions));
    vMode.setBotAng(mode(angles));
    vMean = BotSim(modifiedMap);
    vMean.setScanConfig(vMean.generateScanConfig(scans));
    vMean.setBotPos(mean(positions));
    vMean.setBotAng(mean(angles));
 
    
    if botSim.debug()
        figure(1)
        hold off; 
        botSim.drawMap();
        botSim.drawBot(30,'g');
        for i =1:ParticleCount
            particles(i).drawBot(3);
        end
        vMean.drawBot(30, 'r');
        vMode.drawBot(30, 'b');
        drawnow;
    end 
    
    %% Write code to check for convergence   
    stdev = std(positions);
    convergence_threshold = 2;
    
    if stdev < convergence_threshold
        break; 
    end
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	   
    mutation_rate=0.01; 
    for i=1:mutation_rate*ParticleCount
        particles(randi(ParticleCount)).randomPose(0);
    end 
    
    %% Write code to decide how to move next
   
    if rand()<0.7 
        [max_distance, max_index] = max(botScan); 
        turn = (max_index-1)*2*pi/scans; 
        move = max_distance*0.8*rand(); 
    else 
        index=randi(scans); 
        turn = (index-1)*2*pi/scans;
        move= botScan(index)*0.8;
    end
    botSim.move(move);
    botSim.turn(turn);        

    for i =1:ParticleCount 
          particles(i).turn(turn);
          particles(i).move(move);
    end

    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        figure(1)
        hold off; 
        botSim.drawMap(); 
        botSim.drawBot(30,'g'); 
        for i =1:ParticleCount
            particles(i).drawBot(3); 
        end
        drawnow;
    end
    
end
% Better Check of orientation

botScan = botSim.ultraScan();
diff_mean= zeros(360,1);
diff_mode= zeros(360,1);
for i=1:360  
    vMeanScan = vMean.ultraScan();
    vModeScan = vMode.ultraScan();
    diff_mean(i) = norm(vMeanScan-botScan);
    diff_mode(i) = norm(vModeScan-botScan);
    vMean.setBotAng(i*pi/180);
    vMode.setBotAng(i*pi/180);
end

[min_diff_mean, min_pos_mean] = min(diff_mean);
vMean.setBotAng(min_pos_mean*pi/180); 

[min_diff_mode, min_pos_mode]=min(diff_mode);
vMode.setBotAng(min_pos_mode*pi/180);

if min_diff_mean < min_diff_mode
    virtualBot = vMean;
else
    virtualBot = vMode;
end

end

