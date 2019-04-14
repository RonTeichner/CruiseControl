function csAllModels = CruiseParams(fs,gears,enableLinear)
% function sParams = CruiseParams(fs)
%
% Inputs:
% fs - sample rate [hz]

sModelParams.alpha_n = [40 25 16 12 10];
sModelParams.m = 1200; % [kg]
sModelParams.Cr = 0.01; 
sModelParams.Cd = 0.32;
sModelParams.A = 2.4; % [m^2]
sModelParams.Tm = 190; % [Nm]
sModelParams.omega_m = 420; %[rad/sec]
sModelParams.g = 9.8; % [m/sec^2]

% noises:
sModelParams.std_e = 0;%2*1000/60/60; % [m/s]
% this is not a good noise
sModelParams.std_b = 1e-2*1000/60/60; % [m/s]

% snr of 10db for a speed of 80kph:
snrDb = 10; % [db]
speedPower = (80*1000/60/60)^2;
snrLin = 10^(snrDb / 10);
noisePowerLin = speedPower/snrLin;
noiseStd = sqrt(noisePowerLin);
sModelParams.speedMeasure_std = noiseStd; %0.25*1000/60/60; % [m/s]


%evaluatedSpeedSnr = 10*log10(speedPower / mean((noiseStd * randn(1,1e4)).^2));
%disp(['evaluated speed snr is: ',int2str(evaluatedSpeedSnr),' db']);

% snr of -100db for an accumulated error of 1000 [m]:
snrDb = -100; % [db]
accumulatedErrorPower = 1000^2;
snrLin = 10^(snrDb / 10);
noisePowerLin = accumulatedErrorPower/snrLin;
noiseStd = sqrt(noisePowerLin);
sModelParams.controllerStateMeasure_std = noiseStd; %1e-3; % [m]

%evaluatedaccumulatedErrorSnr = 10*log10(accumulatedErrorPower / mean((noiseStd * randn(1,1e4)).^2));
%disp(['evaluated accumulated Error snr is: ',int2str(evaluatedaccumulatedErrorSnr),' db']);

% controller:
%Lets say we want 90% full gas if we reached 20kph less than Vref
e = 30*1000/60/60;  
sModelParams.Kp = 0.9/e;

% if I am 20 sec at 30kph less I'll add 25%
e = 30*1000/60/60 * 20;
sModelParams.Ki = 0.25/e;

if enableLinear
    sModelParams.beta = 0;
    sModelParams.rho = 0;
else
    sModelParams.beta = 0.4;
    sModelParams.rho = 1.3; %[kg/m^3]
end

nModels = numel(gears);
for i=1:nModels
    csAllModels{i} = sModelParams;
    csAllModels{i}.gear = gears(i);
end
end

