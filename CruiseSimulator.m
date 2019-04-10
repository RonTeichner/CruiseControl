function CruiseSimulator(sSimParams,sModelParams,roadX,sin_theta,roadZ)

vRef = sSimParams.vNominal; % [m/s]
gear = 2;
ts = 1/sSimParams.fs;

initStateVec(1) = sSimParams.vNominal;
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
        
    sinTheta = interp1(roadX,sin_theta,pos(i-1));
    
    
    input_u = [vRef ; sinTheta];
    % gear change:
    %speed_kph = currentStateVec.v*60*60/1000;
    %gear = min(max(ceil(speed_kph/20) - 0 , 1),5);
    
    [nextStateVec,u_k] = CruiseTimeStep(currentStateVec, input_u, sModelParams, sSimParams, gear, b_k, sSimParams.enableLinear);
    
    stateVec(1,i) = nextStateVec(1);
    stateVec(2,i) = nextStateVec(2);
    u(i) = u_k;
    pos(i) = pos(i-1) + nextStateVec(1)*ts;
end

%% figures
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



end

