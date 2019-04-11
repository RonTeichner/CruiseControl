clear; close all; clc;
newScenario = false;
if newScenario
    vNominal_kph = 80; % [kph]
    kph2m_s = 1000/60/60;
    vNominal = vNominal_kph*kph2m_s; % [m/s]
    
    sSimParams.fs = 100; % [hz]
    sSimParams.simDuration = 30*60; % [sec]
    sSimParams.simDistance = sSimParams.simDuration * (2*vNominal); % [m]
    sSimParams.enableLinear = true;
    sSimParams.gear = 2;
    ts = 1/sSimParams.fs;
    
    [roadX,roadZ,sin_theta] = RoadGenerator(sSimParams.simDistance);
    
    roadZ = zeros(size(roadX)); sin_theta = zeros(size(roadX));
    
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

xPlusMean_init = [vNominal_kph * kph2m_s; 0];
xPlusCov_init = [(5*kph2m_s)^2 , 0 ; 0 , 50^2];

nSamples = size(y,2);

xMinusMean  = zeros(size(sKalmanMatrices.F,2) , nSamples);
xMinusCov   = zeros(size(sKalmanMatrices.F,2) , size(sKalmanMatrices.F,2) , nSamples);
xPlusMean  = zeros(size(sKalmanMatrices.F,2) , nSamples);
xPlusCov   = zeros(size(sKalmanMatrices.F,2) , size(sKalmanMatrices.F,2) , nSamples);
K = zeros(size(xPlusCov));
I = eye(2);
for i = 1:nSamples
    % predictor:
    if i > 1
        xMinusMean(:,i) = sKalmanMatrices.F * xPlusMean(:,i-1) + sKalmanMatrices.G * input_u(:,i-1);
        xMinusCov(:,:,i) = sKalmanMatrices.F * xPlusCov(:,:,i-1) * transpose(sKalmanMatrices.F) + sKalmanMatrices.Q;
    else % first iteration from init values
        xMinusMean(:,i) = sKalmanMatrices.F * xPlusMean_init + sKalmanMatrices.G * [0;0];
        xMinusCov(:,:,i) = sKalmanMatrices.F * xPlusCov_init * transpose(sKalmanMatrices.F) + sKalmanMatrices.Q;
    end
    
    % symmetrize P:
    xMinusCov(:,:,i) = 0.5 * (xMinusCov(:,:,i) + transpose(xMinusCov(:,:,i)));
    
    
    % corrector:
    K(:,:,i) = (xMinusCov(:,:,i) * transpose(sKalmanMatrices.C))...
        /(sKalmanMatrices.C * xMinusCov(:,:,i) * transpose(sKalmanMatrices.C) + sKalmanMatrices.R);
    
    xPlusMean(:,i) = xMinusMean(:,i) + K(:,:,i)*(y(:,i) - sKalmanMatrices.C * xMinusMean(:,i));
    xPlusCov(:,:,i) = (I - K(:,:,i)*sKalmanMatrices.C) * xMinusCov(:,:,i) * transpose(I - K(:,:,i)*sKalmanMatrices.C)...
        + K(:,:,i)*sKalmanMatrices.R*transpose(K(:,:,i));
    
    % symmetrize P:
    xPlusCov(:,:,i) = 0.5 * (xPlusCov(:,:,i) + transpose(xPlusCov(:,:,i)));
end

%% ANALYZE
figure;
subplot(2,1,1); hold all;
plot(sGroundTruth.tVec , sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
errorbar(y_tVec , xPlusMean(1,:)./kph2m_s , sqrt(squeeze(xPlusCov(1,1,:)))./kph2m_s );
title('filtered speed'); legend('true','filtered');

subplot(2,1,2); hold all;
plot(sGroundTruth.tVec , sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
errorbar(y_tVec , xPlusMean(2,:) , sqrt(squeeze(xPlusCov(2,2,:))));
title('filtered control-z'); legend('true','filtered');
