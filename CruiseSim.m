clear; close all; clc;

sSimParams.sSimParams.vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
sSimParams.vNominal = sSimParams.sSimParams.vNominal_kph*kph2m_s; % [m/s]
sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 30*60; % [sec]
sSimParams.simDistance = sSimParams.simDuration * (2*sSimParams.vNominal); % [m]
sSimParams.enableLinear = true;
ts = 1/sSimParams.fs;

[roadX,roadZ,sin_theta] = RoadGenerator(sSimParams.simDistance);

sModelParams = CruiseParams(sSimParams.fs , sSimParams.enableLinear);
disp('starting cruise simulator')
CruiseSimulator(sSimParams,sModelParams,roadX,sin_theta,roadZ);


