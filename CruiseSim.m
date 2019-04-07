clear; close all; clc;

vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]
sampleDist = 5; % [m]
sSimParams.fs = 1/(sampleDist/vNominal); % [hz]
sSimParams.simDuration = 100*60; % [sec]
sSimParams.theta_allOnesN = 40;
sSimParams.thetaStd = 5/360*2*pi; % [rad]
sModelParams = CruiseParams(sSimParams.fs);

vRef = vNominal; % [m/s]
gear = 2;
sInitStateVec.v = vNominal;
sInitStateVec.z = 0;

nSamplesInSim = round(sSimParams.simDuration * sSimParams.fs);

%aloc:
sStateVec.v = zeros(nSamplesInSim,1);
sStateVec.z = zeros(nSamplesInSim,1);
u = zeros(nSamplesInSim,1);
theta = zeros(nSamplesInSim,1); % rad

sStateVec.v(1) = sInitStateVec.v;
sStateVec.z(1) = sInitStateVec.z;

for i=2:nSamplesInSim
    currentStateVec.v = sStateVec.v(i-1);
    currentStateVec.z = sStateVec.z(i-1);
    
    % measurement noise:
    e_k = sModelParams.std_e * randn; % [m/s]
    % wind noise:
    b_k = sModelParams.std_b * randn; % [m/s]
    % angle:
    theta_k = mean(theta( max(1,i-sSimParams.theta_allOnesN) : i-1)) + sSimParams.thetaStd*randn;
    theta_k = min(20/360*2*pi , max(-20/360*2*pi , theta_k));
    theta(i) = theta_k;
    % gear change:
    speed_kph = currentStateVec.v*60*60/1000;
    %gear = min(max(ceil(speed_kph/20) - 0 , 1),5);
    
    [sNextStateVec,u_k] = CruiseTimeStep(currentStateVec, vRef, sModelParams,sSimParams, theta_k, gear, e_k, b_k);
    
    sStateVec.v(i) = sNextStateVec.v;
    sStateVec.z(i) = sNextStateVec.z;
    u(i) = u_k;
end

pos = cumsum(sStateVec.v)./sSimParams.fs;

tVec = [0:(nSamplesInSim-1)]./sSimParams.fs;
figure; 
v_kph = sStateVec.v*60*60./1000;
subplot(4,1,1); plot(tVec,v_kph); xlabel('sec'); grid on; ylabel('kph');
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on; 
subplot(4,1,2); plot(tVec,theta/(2*pi)*360); xlabel('sec'); ylabel('angle'); grid on; 
subplot(4,1,3); plot(tVec,u); xlabel('sec'); ylabel('u'); grid on;
subplot(4,1,4); plot(tVec,sStateVec.z); xlabel('sec'); ylabel('z'); grid on;