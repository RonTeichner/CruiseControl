function [ySampleRate,yD_tVec,yD,input_uD,sGroundTruth] = CruiseSimulator(sSimParams,sModelParams,sInputs)

roadX = sInputs.sRoad.roadX; sin_theta = sInputs.sRoad.sin_theta; roadZ = sInputs.sRoad.roadZ;

vRef = sInputs.vRef; % [m/s]
gear = sModelParams.gear;
ts = 1/sSimParams.fs;

initStateVec(1) = vRef;
initStateVec(2) = 0;

nSamplesInSim = round(sSimParams.simDuration * sSimParams.fs);
tVec = [0:(nSamplesInSim-1)]*ts;
%aloc:
stateVec = zeros(2,nSamplesInSim);
u = zeros(nSamplesInSim,1);
theta = zeros(nSamplesInSim,1); % rad
pos = zeros(nSamplesInSim,1); 
input_u = zeros(2,nSamplesInSim); 

stateVec(1,1) = initStateVec(1);
stateVec(2,1) = initStateVec(2);

for i=2:nSamplesInSim
    currentStateVec(1,1) = stateVec(1,i-1);
    currentStateVec(2,1) = stateVec(2,i-1);
    
    % measurement noise:
    e_k = sModelParams.std_e * randn; % [m/s]
    % wind noise:
    b_k = sModelParams.std_b * randn; % [m/s]
        
    sinTheta = interp1(roadX,sin_theta,pos(i-1));
    
    
    input_u(:,i-1) = [vRef ; sinTheta];
    % gear change:
    %speed_kph = currentStateVec.v*60*60/1000;
    %gear = min(max(ceil(speed_kph/20) - 0 , 1),5);
    
    [nextStateVec,u_k] = CruiseTimeStep(currentStateVec, input_u(:,i-1), sModelParams, sSimParams, gear, b_k, sSimParams.enableLinear);
    
    stateVec(1,i) = nextStateVec(1);
    stateVec(2,i) = nextStateVec(2);
    u(i) = u_k;
    pos(i) = pos(i-1) + nextStateVec(1)*ts;
end
sGroundTruth.stateVec   = stateVec;
sGroundTruth.tVec       = tVec;
sGroundTruth.u          = u;
sGroundTruth.pos = pos;
%% observer
C = [1,0;0,1];

observationNoise = [sModelParams.speedMeasure_std * randn(1,size(stateVec,2));...
    sModelParams.controllerStateMeasure_std * randn(1,size(stateVec,2))];

y = C*stateVec + observationNoise;

% it seems resnable to downsample y.
desired_ySampleRate = 1; % [hz]
yDownSampleRate = round(sSimParams.fs/desired_ySampleRate);
ySampleRate = sSimParams.fs / yDownSampleRate;

% downsample:
yD(1,:) = y(1,1:yDownSampleRate:end);
yD(2,:) = y(2,1:yDownSampleRate:end);
yD_tVec = tVec(1) + [0:(size(yD,2)-1)]./ySampleRate;

input_uD(1,:) = input_u(1,1:yDownSampleRate:end);
input_uD(2,:) = input_u(2,1:yDownSampleRate:end);





end

