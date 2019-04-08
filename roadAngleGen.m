function [x,sin_theta] = roadAngleGen(maxSlope,distance)
% Inputs:
% maxSlope [deg/m]

ts = 1; % sample every 'ts' meters.
fs = 1/ts; % [hz]

cutOffFreq = maxSlope/360;
Wn = cutOffFreq / (fs/2);

N = 10;
b = fir1(N,Wn);
%  B = fir1(N,Wn);  Wn must be between 0 < Wn < 1.0, with 1.0 corresponding to half the sample rate.
%freqz(b,1,[],fs);

nSamples = distance*fs;
x = [0:(nSamples - 1)]./fs; % [m]

whiteNoise = randn(size(x));
theta = filter(b,1,whiteNoise); % [deg]
theta_rad = (theta./360)*2*pi;
sin_theta = sin(theta_rad);
end

