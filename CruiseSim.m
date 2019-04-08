clear; close all; clc;

vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]
sampleDist = 5; % [m]
% sSimParams.fs = 1/(sampleDist/vNominal); % [hz]
sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 1*60; % [sec]

ts = 1/sSimParams.fs;

maxSlope = atan(1/5)/(2*pi)*360; % [deg]
distance = sSimParams.simDuration * (200*kph2m_s); % [m]
[x,sin_theta] = roadAngleGen(maxSlope,distance);

enableLinear = true;
sModelParams = CruiseParams(sSimParams.fs , enableLinear);

vRef = vNominal; % [m/s]
gear = 2;
initStateVec(1) = 0;%vNominal;
initStateVec(2) = 0;

nSamplesInSim = round(sSimParams.simDuration * sSimParams.fs);

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
        
    sinTheta = interp1(x,sin_theta,pos(i-1));
    input_u = [vRef ; sinTheta];
    % gear change:
    %speed_kph = currentStateVec.v*60*60/1000;
    %gear = min(max(ceil(speed_kph/20) - 0 , 1),5);
    
    [nextStateVec,u_k] = CruiseTimeStep(currentStateVec, input_u, sModelParams, sSimParams, gear, b_k);
    
    stateVec(1,i) = nextStateVec(1);
    stateVec(2,i) = nextStateVec(2);
    u(i) = u_k;
    pos(i) = pos(i-1) + nextStateVec(1)*ts;
end

tVec = [0:(nSamplesInSim-1)]./sSimParams.fs;
figure; 
v_kph = stateVec(1,:)*60*60./1000;
subplot(4,1,1); plot(tVec,v_kph); xlabel('sec'); grid on; ylabel('kph');
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on; 
subplot(4,1,2); plot(tVec,interp1(x,asin(sin_theta)/(2*pi)*360,pos)); xlabel('sec'); ylabel('angle'); grid on; 
subplot(4,1,3); plot(tVec,u); xlabel('sec'); ylabel('u'); grid on;
subplot(4,1,4); plot(tVec,stateVec(2,:)); xlabel('sec'); ylabel('z'); grid on;

figure;
plot(pos ,interp1(x,asin(sin_theta)/(2*pi)*360,pos)); xlabel('m'); ylabel('deg'); title('road'); grid on;