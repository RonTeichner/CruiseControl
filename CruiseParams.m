function sParams = CruiseParams(fs)
% function sParams = CruiseParams(fs)
%
% Inputs:
% fs - sample rate [hz]

sParams.alpha_n = [40 25 16 12 10];
sParams.m = 1200e3; % [gram]
sParams.Cr = 0.01; 
sParams.rho = 1.3e3; %[gram/m^3]
sParams.Cd = 0.32;
sParams.A = 2.4; % [m^2]
sParams.Tm = 190; % [Nm]
sParams.omega_m = 420; %[rad/sec]
sParams.beta = 0.4;
sParams.g = 9.8; % [m/sec^2]

% noises:
sParams.std_e = 2*1000/60/60; % [m/s]
% this is not a good noise
sParams.std_b = 1*1000/60/60; % [m/s]

% controller:
sParams.Kp = 1;
sParams.Ki = 1;

end

