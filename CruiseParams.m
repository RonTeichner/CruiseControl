function sParams = CruiseParams(fs,enableLinear)
% function sParams = CruiseParams(fs)
%
% Inputs:
% fs - sample rate [hz]

sParams.alpha_n = [40 25 16 12 10];
sParams.m = 1200; % [kg]
sParams.Cr = 0.01; 
sParams.Cd = 0.32;
sParams.A = 2.4; % [m^2]
sParams.Tm = 190; % [Nm]
sParams.omega_m = 420; %[rad/sec]
sParams.g = 9.8; % [m/sec^2]

% noises:
sParams.std_e = 0;%2*1000/60/60; % [m/s]
% this is not a good noise
sParams.std_b = 0*1000/60/60; % [m/s]

% controller:
sParams.Kp = 0.3;
sParams.Ki = 0.7;

if enableLinear
    sParams.beta = 0;
    sParams.rho = 0;
else
    sParams.beta = 0.4;
    sParams.rho = 1.3; %[kg/m^3]
end

end

