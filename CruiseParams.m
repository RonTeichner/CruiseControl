function csAllModels = CruiseParams(fs,gears,enableLinear)
% function sParams = CruiseParams(fs)
%
% Inputs:
% fs - sample rate [hz]

sModelParams.alpha_n = [40 25 16 12 10];
sModelParams.m = 1200; % [kg]
sModelParams.Cr = 0.01; % not in use
sModelParams.Cd = 0.32;
sModelParams.A = 2.4; % [m^2]
sModelParams.Tm = 40; % 190 [Nm] % in 190 at non-linear example with small slopes the car didn't change gears because of rather constant speed
sModelParams.omega_m = 420; %[rad/sec]
sModelParams.g = 9.8; % [m/sec^2]

% noises:
sModelParams.std_e = 0;%2*1000/60/60; % [m/s]
% this is not a good noise
sModelParams.std_b = 1e-3*1000/60/60; % [m/s]

% snr of 10db for a speed of 80kph:
snrDb = 16; % [db]
speedPower = (60*1000/60/60); % I get old and think it is not correct to square this value (80*1000/60/60)^2;
snrLin = 10^(snrDb / 10);
noisePowerLin = speedPower/snrLin;
noiseStd = noisePowerLin; %sqrt(noisePowerLin);
sModelParams.speedMeasure_std = noiseStd; %0.25*1000/60/60; % [m/s]


%evaluatedSpeedSnr = 10*log10(speedPower / mean((noiseStd * randn(1,1e4)).^2));
%disp(['evaluated speed snr is: ',int2str(evaluatedSpeedSnr),' db']);

% snr of -100db for an accumulated error of 1000 [m]:
snrDb = 16; % [db]
accumulatedErrorPower = 200; % I get old and think it is not correct to square this value 200^2;
snrLin = 10^(snrDb / 10);
noisePowerLin = accumulatedErrorPower/snrLin;
noiseStd = noisePowerLin; %sqrt(noisePowerLin);
sModelParams.controllerStateMeasure_std = noiseStd; %1e-3; % [m]

%evaluatedaccumulatedErrorSnr = 10*log10(accumulatedErrorPower / mean((noiseStd * randn(1,1e4)).^2));
%disp(['evaluated accumulated Error snr is: ',int2str(evaluatedaccumulatedErrorSnr),' db']);

% controller:
%Lets say we want 90% full gas if we reached 20kph less than Vref
e = 30*1000/60/60;  
sModelParams.Kp = 0.9/e;

% if I am 20 sec at 30kph less I'll add 35%
e = 30*1000/60/60 * 20;
sModelParams.Ki = 0.35/e;

if enableLinear
    sModelParams.beta = 0;
    sModelParams.rho = 0;
else
    sModelParams.beta = 0.4;
    sModelParams.rho = 1.3; %[kg/m^3]
end


% transitionMat(r,c) is the chance of changing from gear r to gear c
% transitionMat = [ ...
%     [0 ; 0.5 ; 0.24 ; 0.20 ; 0.06] ,...
%     [0.35 ; 0 ; 0.35 ; 0.20 ; 0.10],...
%     [0.10 ; 0.35 ; 0 ; 0.35 ; 0.20],...
%     [0.10 ; 0.20 ; 0.35 ; 0 ; 0.35],...
%     [0.06 ; 0.20 ; 0.24 ; 0.5 ; 0]...
%     ];

transitionMat = [ ...
    [0 ; 0.8 ; 0.10 ; 0.08 ; 0.02] ,...
    [0.45 ; 0 ; 0.45 ; 0.08 ; 0.02],...
    [0.10 ; 0.45 ; 0 ; 0.45 ; 0.10],...
    [0.02 ; 0.08 ; 0.45 ; 0 ; 0.45],...
    [0.02 ; 0.08 ; 0.10 ; 0.8 ; 0]...
    ];



sModelParams.transitionMat = transitionMat;

nModels = numel(gears);
for i=1:nModels
    csAllModels{i} = sModelParams;
    csAllModels{i}.gear = gears(i);
end
end

