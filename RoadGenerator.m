function [posVec,roadZAtPosVec,sin_theta_atPosVec] = RoadGenerator(simDistance)
distance = 1000e3; % [m]
peakHeight = 500;%300; % [m]
enableFig = true;
[roadX,sin_theta,roadZ] = roadAngleGen(peakHeight,distance,enableFig);

peakHeight = 10; % [m]
[roadX2,sin_theta2,roadZ2] = roadAngleGen(peakHeight,distance,~enableFig);

% switch from mountain to plane every 10 Km:
switchEvery = 10e3; % [m]
roadX_ts = mean(diff(roadX));
nSamplesIn10Km = switchEvery/roadX_ts;
%roadX = reshape(roadX,nSamplesIn10Km,[]);
sin_theta = reshape(sin_theta,nSamplesIn10Km,[]);
roadZ = reshape(roadZ,nSamplesIn10Km,[]);

%roadX2 = reshape(roadX2,nSamplesIn10Km,[]);
sin_theta2 = reshape(sin_theta2,nSamplesIn10Km,[]);
roadZ2 = reshape(roadZ2,nSamplesIn10Km,[]);

%roadX = reshape([roadX ; roadX2],[],1);
sin_theta = reshape([sin_theta ; sin_theta2],[],1);
roadZ = reshape([roadZ ; roadZ2],[],1);
roadX = [0:(numel(roadZ)-1)]*roadX_ts;

posVecFs = 0.5; % [m]
nSamples = ceil(simDistance*posVecFs);
posVec = [0:(nSamples-1)]./posVecFs;

roadZAtPosVec = transpose(interp1(roadX,roadZ,posVec,'spline'));

theta_rad = atan([0 ; diff(roadZAtPosVec)]*posVecFs);
sin_theta_atPosVec = sin(theta_rad);
end

