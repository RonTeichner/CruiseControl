clear; close all; clc;

vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]
sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 30*60; % [sec]

ts = 1/sSimParams.fs;


distance = 1000e3; % [m]
peakHeight = 300; % [m]
[roadX,sin_theta,roadZ] = roadAngleGen(peakHeight,distance);

peakHeight = 10; % [m]
[roadX2,sin_theta2,roadZ2] = roadAngleGen(peakHeight,distance);

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
% roadX = [0 ; 100e3];
% sin_theta = 10*[1/360*2*pi ; 1/360*2*pi];
% roadZ = [0 ; tan(sin_theta(1))*diff(roadX)];


enableLinear = true;
sModelParams = CruiseParams(sSimParams.fs , enableLinear);

vRef = vNominal; % [m/s]
gear = 2;
initStateVec(1) = vNominal;
initStateVec(2) = 0;

nSamplesInSim = round(sSimParams.simDuration * sSimParams.fs);
tVec = [0:(nSamplesInSim-1)]*ts;
%aloc:
stateVec = zeros(2,nSamplesInSim);
u = zeros(nSamplesInSim,1);
theta = zeros(nSamplesInSim,1); % rad
pos = zeros(nSamplesInSim,1); 

stateVec(1,1) = initStateVec(1);
stateVec(2,1) = initStateVec(2);

for i=2:nSamplesInSim
    currentStateVec(1,1) = stateVec(1,i-1);
    currentStateVec(2,1) = stateVec(2,i-1);
    
    % measurement noise:
    e_k = sModelParams.std_e * randn; % [m/s]
    % wind noise:
    b_k = sModelParams.std_b * randn; % [m/s]
        
    sinTheta = interp1(roadX,sin_theta,pos(i-1),'spline');
    input_u = [vRef ; sinTheta];
    % gear change:
    %speed_kph = currentStateVec.v*60*60/1000;
    %gear = min(max(ceil(speed_kph/20) - 0 , 1),5);
    
    [nextStateVec,u_k] = CruiseTimeStep(currentStateVec, input_u, sModelParams, sSimParams, gear, b_k, enableLinear);
    
    stateVec(1,i) = nextStateVec(1);
    stateVec(2,i) = nextStateVec(2);
    u(i) = u_k;
    pos(i) = pos(i-1) + nextStateVec(1)*ts;
end


roadZ_atPos = interp1(roadX,roadZ,pos,'spline');
thetaDeg_atPos = asin(interp1(roadX,sin_theta,pos,'spline'))/2/pi*360;
figure; 
v_kph = stateVec(1,:)*60*60./1000;
subplot(4,1,1); plot(tVec,v_kph); xlabel('sec'); grid on; ylabel('kph');
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on; 
subplot(4,1,2); plot(tVec,roadZ_atPos); xlabel('sec'); ylabel('m'); title('roadZ'); grid on; 
subplot(4,1,3); plot(tVec,u); xlabel('sec'); ylabel('u'); grid on;
subplot(4,1,4); plot(tVec,stateVec(2,:)); xlabel('sec'); ylabel('z'); grid on;

figure;
subplot(2,1,1); plot(pos,roadZ_atPos); xlabel('m'); ylabel('m'); title('road'); grid on;
subplot(2,1,2); plot(pos,thetaDeg_atPos); xlabel('m'); ylabel('deg'); title('road sloape'); grid on;