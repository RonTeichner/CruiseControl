clear; close all; clc;

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
sInputs.sRoad.roadX = roadX; sInputs.sRoad.roadZ = roadZ; sInputs.sRoad.sin_theta = sin_theta;
sInputs.vRef = vNominal;
sModelParams = CruiseParams(sSimParams.fs , sSimParams.enableLinear);
disp('starting cruise simulator')
[y_fs,y_tVec,y] = CruiseSimulator(sSimParams,sModelParams,sInputs);

figure; 
subplot(2,1,1); plot(y_tVec , y(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); title('measured speed'); grid on;
subplot(2,1,2); plot(y_tVec , y(2,:)); xlabel('sec'); ylabel('m'); title('measured cumulative error'); grid on;


sKalmanMatrices = CruiseKalmanParams(sModelParams,sSimParams.gear,y_fs);

% run Kalman filter:
xPlusMean_init = [vNominal_kph * kph2m_s; 0];
xPlusCov_init = [(5*kph2m_s)^2 , 0 ; 0 , 50^2];
