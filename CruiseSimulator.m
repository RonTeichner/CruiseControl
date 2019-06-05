function [ySampleRate,yD_tVec,yD,input_uD,gearChangeD,sGroundTruth] = CruiseSimulator(sSimParams,sModelParams,sInputs)

roadX = sInputs.sRoad.roadX; sin_theta = sInputs.sRoad.sin_theta;

vRef = sInputs.vRef; % [m/s]
ts = 1/sSimParams.fs;

% if sSimParams.enableGearChange
%     initStateVec(1) = 0;
%     gear = 1;
% else
initStateVec(1) = 0;%vRef;
gear = 1;%sModelParams.gear;
% end
initStateVec(2) = 0;

nSamplesInSim = round(sSimParams.simDuration * sSimParams.fs);
tVec = [0:(nSamplesInSim-1)]*ts;
%aloc:
stateVec = zeros(2,nSamplesInSim);
u = zeros(nSamplesInSim,1);
gears = zeros(nSamplesInSim,1);
pos = zeros(nSamplesInSim,1);
input_u = zeros(2,nSamplesInSim);
stateVec(1,1) = initStateVec(1);
stateVec(2,1) = initStateVec(2);

% gear change logic:
% min time between gear change is 4 seconds
minTimeBetweenGearChanges = 4;%15; % [sec]
minSpeedDiffBetweenGearChanges = 10*1000/60/60; % [m/s]
% we change the gears at certain speed when the speed growth and in
% certain speed when speeds lowers:
speedUpLimits   = [ 20 , 40 , 60 , 80 ]*1000./60./60; % at 20kph from 1 to 2, at 40 from 2 to 3...
speedDownLimits = [ 10 , 20 , 30 , 60 ]*1000./60./60; % at 60 from 5 to 4, at 30 from 4 to 3, at 20 from 3 to 2 at 10 from 2 to 1
gears(1) = gear;
previousStateVec = [0;0];
previousGearChangeTime = -inf; previousGearChangeSpeed = 0;
for i=2:nSamplesInSim
    currentTime = tVec(i);
    currentStateVec(1,1) = stateVec(1,i-1);
    currentStateVec(2,1) = stateVec(2,i-1);
    
    % measurement noise:
    e_k = sModelParams.std_e * randn; % [m/s]
    % wind noise:
    b_k = sModelParams.std_b * randn; % [m/s]
    
    sinTheta = interp1(roadX,sin_theta,pos(i-1));
    
    
    input_u(:,i-1) = [vRef ; sinTheta];
    
    % gear change:
    if sSimParams.enableGearChange
        
        if currentStateVec(1) - previousGearChangeSpeed > minSpeedDiffBetweenGearChanges
            speedDirection = 1;
        elseif previousGearChangeSpeed - currentStateVec(1) > minSpeedDiffBetweenGearChanges
            speedDirection = -1;
        else
            speedDirection = 0;
        end
        
        if previousGearChangeTime + minTimeBetweenGearChanges < currentTime
            if abs(currentStateVec(1) - previousGearChangeSpeed) > minSpeedDiffBetweenGearChanges
                if speedDirection == 1 % speed up
                    if currentStateVec(1,1) >= speedUpLimits(end)
                        gear = 5;
                    else
                        gear = find(speedUpLimits > currentStateVec(1,1) , 1);
                    end
                elseif speedDirection == -1 % speed down
                    if currentStateVec(1,1) <= speedDownLimits(1)
                        gear = 1;
                    else
                        gear = 1 + find(speedDownLimits < currentStateVec(1,1) ,1,'last');
                    end
                end
            end
        end
    end
    
    [nextStateVec,u_k] = CruiseTimeStep(currentStateVec, input_u(:,i-1), sModelParams, sSimParams, gear, b_k, sSimParams.enableLinear);
    previousStateVec = currentStateVec;
    
    stateVec(1,i) = nextStateVec(1);
    stateVec(2,i) = nextStateVec(2);
    u(i) = u_k;
    gears(i) = gear;
    if gear ~= gears(i-1)
        previousGearChangeTime = currentTime;
        previousGearChangeSpeed = currentStateVec(1);
    end
    pos(i) = pos(i-1) + nextStateVec(1)*ts;
    
    if i==2
        figure;
    end
    if mod(i,1e3)==0
        hold on; subplot(3,1,1);    plot(tVec(1:i), stateVec(1,1:i)*60*60/1000); xlabel('sec'); ylabel('kph');
        subplot(3,1,2);             plot(tVec(1:i), u(1:i)); xlabel('sec'); ylabel('u');
        subplot(3,1,3);             plot(tVec(1:i-1), asin(input_u(2,1:i-1))./pi*180); xlabel('sec'); ylabel('slope [deg]'); 
        pause(0.1);
        %display(['speed: ',num2str(nextStateVec(1)*60*60/1000),' kph; gear: ',int2str(gear)]);
    end
    
end
sGroundTruth.stateVec   = stateVec;
sGroundTruth.tVec       = tVec;
sGroundTruth.u          = u;
sGroundTruth.gears      = gears;
sGroundTruth.pos = pos;

%% observer
C = [1,0;0,1];

observationNoise = [sModelParams.speedMeasure_std * randn(1,size(stateVec,2));...
    sModelParams.controllerStateMeasure_std * randn(1,size(stateVec,2))];

yNoNoise = C*stateVec;
y = yNoNoise + observationNoise;

meanObserverNoNoisePower = mean(transpose(yNoNoise.^2));
meanObserverNoisePower = mean(transpose(observationNoise.^2));

disp(['speed & controller measure snr: ',mat2str(10*log10(meanObserverNoNoisePower ./ meanObserverNoisePower)),' db']);

% it seems resnable to downsample y.
%desired_ySampleRate = 1; % [hz]
yDownSampleRate = round(sSimParams.fs/sSimParams.desired_ySampleRate);
ySampleRate = sSimParams.fs / yDownSampleRate;

% downsample:
yD(1,:) = y(1,1:yDownSampleRate:end);
yD(2,:) = y(2,1:yDownSampleRate:end);
yD_tVec = tVec(1) + [0:(size(yD,2)-1)]./ySampleRate;

input_uD(1,:) = input_u(1,1:yDownSampleRate:end);
input_uD(2,:) = input_u(2,1:yDownSampleRate:end);

gearChangeD = zeros(size(yD_tVec));
for i=2:numel(gearChangeD)
    startPeriodTime = yD_tVec(i-1);
    endPeriodTime = yD_tVec(i);
    
    firstIdxAt_tVec = find(tVec - startPeriodTime > 0 , 1 , 'first');
    lastIdxAt_tVec  = find(endPeriodTime <= tVec , 1 , 'first');
    
    if gears(lastIdxAt_tVec) > gears(firstIdxAt_tVec)
        gearChangeD(i) = 1;
    elseif gears(lastIdxAt_tVec) < gears(firstIdxAt_tVec)
        gearChangeD(i) = -1;
    end
end

sGroundTruth.yTvec = yD_tVec;
sGroundTruth.stateVec_atMeasureTimes(1,:) = yNoNoise(1,1:yDownSampleRate:end);
sGroundTruth.stateVec_atMeasureTimes(2,:) = yNoNoise(2,1:yDownSampleRate:end);
sGroundTruth.gears_atMeasureTimes = gears(1:yDownSampleRate:end);

end

