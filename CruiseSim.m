clear; close all; clc;
newRoad = false;
newScenarios = false;

vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]

sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 30*60; % [sec]
sSimParams.simDistance = max(10e3, sSimParams.simDuration * (2*vNominal)); % [m]
sSimParams.enableLinear = true;
gear = 1:5;
ts = 1/sSimParams.fs;

if newRoad
    [roadX,roadZ,sin_theta] = RoadGenerator(sSimParams.simDistance);
    save('road.mat','roadX','roadZ','sin_theta');
else
    load('road.mat');
end
%roadZ = zeros(size(roadX)); sin_theta = zeros(size(roadX));

sInputs.sRoad.roadX = roadX; sInputs.sRoad.roadZ = roadZ; sInputs.sRoad.sin_theta = sin_theta;
sInputs.vRef = vNominal;

if newScenarios
    csAllModels = CruiseParams(sSimParams.fs , gear, sSimParams.enableLinear);
    for modelIdx = 1:numel(csAllModels)
        disp('starting cruise simulator')
        [y_fs,y_tVec,y,input_u,sGroundTruth] = CruiseSimulator(sSimParams,csAllModels{modelIdx},sInputs);
        
        csSim{modelIdx}.sModelParams = csAllModels{modelIdx};
        csSim{modelIdx}.sGroundTruth = sGroundTruth;
        csSim{modelIdx}.input_u = input_u;
        csSim{modelIdx}.y = y; csSim{modelIdx}.y_tVec = y_tVec; csSim{modelIdx}.y_fs = y_fs;
    end
    clear csAllModels
    save('scenario.mat','csSim')
else
    load('scenario.mat');
end
%% figures
nScenarios = numel(csSim);
for scIdx = 1:nScenarios
    roadZ_atPos{scIdx} = interp1(roadX,roadZ,               csSim{scIdx}.sGroundTruth.pos,'spline');
    thetaDeg_atPos{scIdx} = asin(interp1(roadX,sin_theta,   csSim{scIdx}.sGroundTruth.pos,'spline'))/2/pi*360;
    v_kph{scIdx} = csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s;
end

figure;

subplot(4,1,1); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,v_kph{scIdx}); xlabel('sec'); grid on; ylabel('kph'); title('speed');
end
legend('1','2','3','4','5');
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on;

subplot(4,1,2); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,roadZ_atPos{scIdx}); xlabel('sec'); ylabel('m'); title('vertical position'); grid on;
end
legend('1','2','3','4','5');

subplot(4,1,3); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,csSim{scIdx}.sGroundTruth.u); xlabel('sec'); title('u'); grid on;
end
legend('1','2','3','4','5');

subplot(4,1,4); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); title('z'); ylabel('m'); grid on;
end
legend('1','2','3','4','5');

figure; hold all;
groundTruthTs = csSim{scIdx}.sGroundTruth.tVec(2)-csSim{scIdx}.sGroundTruth.tVec(1);
for scIdx = 1:nScenarios
    speedAtPos = ([0 ; diff(csSim{scIdx}.sGroundTruth.pos)]./groundTruthTs)./kph2m_s;
    plot(csSim{scIdx}.sGroundTruth.pos , speedAtPos); grid on; xlabel('m'); ylabel('kph'); title('speed vs pos');
end
legend('1','2','3','4','5');

figure;
subplot(2,1,1); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,roadZ_atPos{scIdx}); xlabel('m'); ylabel('m'); title('road'); grid on;
end
legend('1','2','3','4','5');

subplot(2,1,2);  hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,thetaDeg_atPos{scIdx}); xlabel('m'); ylabel('deg'); title('road slope'); grid on;
end
legend('1','2','3','4','5');

figure;
subplot(2,1,1);   hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); title('measured speed'); grid on;
end
legend('1','2','3','4','5');

subplot(2,1,2);   hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(2,:)); xlabel('sec'); ylabel('m'); title('measured cumulative error'); grid on;
end
legend('1','2','3','4','5');

%% run Kalman filter:
y_fs = csSim{1}.y_fs;
for modelIdx = 1:nScenarios
    sKalmanMatrices{modelIdx} = CruiseKalmanParams(csSim{modelIdx}.sModelParams, y_fs, sSimParams.fs);
    % for vRef = 80kph stable u (throttle level) was measured to be 0.5491 when
    % Ki was 0.0015; therefore stable z is 0.5491/0.0015 = 366.0718
    sInitValues{modelIdx}.xPlusMean_init      = [vNominal_kph * kph2m_s; 366.0781];
    sInitValues{modelIdx}.xPlusCov_init       = [(5*kph2m_s)^2 , 0 ; 0 , 50^2];
    sInitValues{modelIdx}.uInput_init         = csSim{modelIdx}.input_u(:,1);
end
% run for every combination of model and scenario:
filteringIdx = 0;
for modelIdx = 1:nScenarios
    for scIdx = 1:nScenarios
        [xPlusMean , xPlusCov , weightFactor] = CruiseKalmanFilter(sKalmanMatrices{modelIdx},sInitValues{modelIdx}, csSim{scIdx}.y, csSim{scIdx}.input_u);
        filteringIdx = filteringIdx + 1;
        
        csKalmanRes{filteringIdx}.sKalmanMatrices   = sKalmanMatrices{modelIdx};
        csKalmanRes{filteringIdx}.sInitValues       = sInitValues{modelIdx};
        csKalmanRes{filteringIdx}.scenarioIdx       = scIdx;
        
        csKalmanRes{filteringIdx}.xPlusMean = xPlusMean; csKalmanRes{filteringIdx}.xPlusCov = xPlusCov; csKalmanRes{filteringIdx}.weightFactor = weightFactor;
    end
end
%% ANALYZE
for filteringIdx = 1:numel(csKalmanRes)
    scIdx = csKalmanRes{filteringIdx}.scenarioIdx;
    figure;
    subplot(2,1,1); hold all;
    errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:)))./kph2m_s );
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    title('filtered speed'); legend('filtered','true');
    
    subplot(2,1,2); hold all;
    errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(2,:) , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(2,2,:))));
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
    title('filtered control-z'); legend('filtered','true');
    
    figure;
    firstSamleToPlot = 100;
    csKalmanRes{filteringIdx}.xPlusCovTrace = zeros(size(csKalmanRes{filteringIdx}.xPlusCov,3),1);
    for i=1:size(csKalmanRes{filteringIdx}.xPlusCov,3)
        csKalmanRes{filteringIdx}.xPlusCovTrace(i) = trace(csKalmanRes{filteringIdx}.xPlusCov(:,:,i));
    end
    subplot(3,1,1);
    plot(csSim{scIdx}.y_tVec(firstSamleToPlot:end) , csKalmanRes{filteringIdx}.xPlusCovTrace(firstSamleToPlot:end)); xlabel('sec'); title('trace of covMat'); grid on;
    subplot(3,1,2);
    plot(csSim{scIdx}.y_tVec(firstSamleToPlot:end) , csKalmanRes{filteringIdx}.weightFactor(firstSamleToPlot:end)); xlabel('sec'); title('weight factor'); grid on;
    subplot(3,1,3);hold all;
    errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:)))./kph2m_s );
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    title('filtered speed'); legend('filtered','true');
end
