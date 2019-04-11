clear; close all; clc;
newScenario = true;
if newScenario
    vNominal_kph = 80; % [kph]
    kph2m_s = 1000/60/60;
    vNominal = vNominal_kph*kph2m_s; % [m/s]
    
    sSimParams.fs = 100; % [hz]
    sSimParams.simDuration = 30*60; % [sec]
    sSimParams.simDistance = max(10e3, sSimParams.simDuration * (2*vNominal)); % [m]
    sSimParams.enableLinear = true;
    sSimParams.gear = 2;
    ts = 1/sSimParams.fs;
    
    [roadX,roadZ,sin_theta] = RoadGenerator(sSimParams.simDistance);
    
    %roadZ = zeros(size(roadX)); sin_theta = zeros(size(roadX));
    
    sInputs.sRoad.roadX = roadX; sInputs.sRoad.roadZ = roadZ; sInputs.sRoad.sin_theta = sin_theta;
    sInputs.vRef = vNominal;
    sModelParams = CruiseParams(sSimParams.fs , sSimParams.enableLinear);
    disp('starting cruise simulator')
    [y_fs,y_tVec,y,input_u,sGroundTruth] = CruiseSimulator(sSimParams,sModelParams,sInputs);
    
    figure;
    subplot(2,1,1); plot(y_tVec , y(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); title('measured speed'); grid on;
    subplot(2,1,2); plot(y_tVec , y(2,:)); xlabel('sec'); ylabel('m'); title('measured cumulative error'); grid on;
    
    save('scenario.mat')
    
else
    load('scenario.mat')
end

%% run Kalman filter:
sKalmanMatrices = CruiseKalmanParams(sModelParams,sSimParams.gear,y_fs,sSimParams.fs);
% for vRef = 80kph stable u (throttle level) was measured to be 0.5491 when
% Ki was 0.0015; therefore stable z is 0.5491/0.0015 = 366.0718
sInitValues.xPlusMean_init      = [vNominal_kph * kph2m_s; 366.0781];
sInitValues.xPlusCov_init       = [(5*kph2m_s)^2 , 0 ; 0 , 50^2];
sInitValues.uInput_init         = input_u(:,1);

[xPlusMean , xPlusCov] = CruiseKalmanFilter(sKalmanMatrices,sInitValues,y,input_u);

%% ANALYZE
figure;
subplot(2,1,1); hold all;
errorbar(y_tVec , xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(xPlusCov(1,1,:)))./kph2m_s );
plot(sGroundTruth.tVec , sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
title('filtered speed'); legend('filtered','true');

subplot(2,1,2); hold all;
errorbar(y_tVec , xPlusMean(2,:) , 3*sqrt(squeeze(xPlusCov(2,2,:))));
plot(sGroundTruth.tVec , sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
title('filtered control-z'); legend('filtered','true');
