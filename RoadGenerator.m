function [posVec,roadZAtPosVec,sin_theta_atPosVec] = RoadGenerator(simDistance)
distance = 1000e3; % [m]

enablePlane = false;

mountainLenght = 10e3; % [m]
planeLenght = 5e3; % [m]

peakHeight = 70; % [m] 
enableFig = false;
[roadX,sin_theta,roadZ] = roadAngleGen(peakHeight,distance,enableFig);

if enablePlane
    peakHeight = 10; % [m]
    [roadX2,sin_theta2,roadZ2] = roadAngleGen(peakHeight,distance*(planeLenght/mountainLenght),~enableFig);
    
    % switch from mountain to plane every 10 Km:
    
    roadX_ts = mean(diff(roadX));
    nSamplesInMountain = mountainLenght/roadX_ts;
    %roadX = reshape(roadX,nSamplesIn10Km,[]);
    sin_theta = reshape(sin_theta,nSamplesInMountain,[]);
    roadZ = reshape(roadZ,nSamplesInMountain,[]);
    
    nSamplesInPlane = planeLenght/roadX_ts;
    %roadX2 = reshape(roadX2,nSamplesIn10Km,[]);
    sin_theta2 = reshape(sin_theta2,nSamplesInPlane,[]);
    roadZ2 = reshape(roadZ2,nSamplesInPlane,[]);
    
    %roadX = reshape([roadX ; roadX2],[],1);
    sin_theta = reshape([sin_theta2 ; sin_theta],[],1);
    roadZ = reshape([roadZ2 ; roadZ],[],1);
    roadX = [0:(numel(roadZ)-1)]*roadX_ts;
end

posVecFs = 0.5; % [m]
nSamples = ceil(simDistance*posVecFs);
posVec = [0:(nSamples-1)]./posVecFs;

roadZAtPosVec = transpose(interp1(roadX,roadZ,posVec,'spline'));

theta_rad = atan([0 ; diff(roadZAtPosVec)]*posVecFs);
sin_theta_atPosVec = sin(theta_rad);
end

