clear;
close all; clc;
newRoad = true;
newScenarios = true;

vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]

sSimParams.nSeed = round(1e7*rand);
sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 30*60; % [sec]
sSimParams.simDistance = max(10e3, sSimParams.simDuration * (2*vNominal)); % [m]
sSimParams.enableLinear = false;
sSimParams.nScenarios = 1;
sSimParams.enableGearChange = true;
sSimParams.returnToInitValueInReset = false;
sSimParams.desired_ySampleRate = 1; % [hz]
sSimParams.doMyFiltering = false;

gear = 1:5;
ts = 1/sSimParams.fs;

if sSimParams.nSeed > 0
    rng(sSimParams.nSeed)
    disp(['setting seed to ',int2str(sSimParams.nSeed)]);
end

%% create road:
if newRoad
    [roadX,roadZ,sin_theta] = RoadGenerator(sSimParams.simDistance);
    save('road.mat','roadX','roadZ','sin_theta');
else
    load('road.mat');
end
%roadZ = zeros(size(roadX)); sin_theta = zeros(size(roadX));

sInputs.sRoad.roadX = roadX; sInputs.sRoad.roadZ = roadZ; sInputs.sRoad.sin_theta = sin_theta;
sInputs.vRef = vNominal;

%sInputs.sRoad.roadZ = zeros(size(sInputs.sRoad.roadZ)); sInputs.sRoad.sin_theta = zeros(size(sInputs.sRoad.roadZ)); 

figure; subplot(2,1,1); plot(roadX./1e3 , roadZ); xlabel('Km'); ylabel('m'); title('terrain');
roadAngle = atan(diff(roadZ) ./ diff(transpose(roadX))); % [rad]
subplot(2,1,2); plot(roadX(2:end)./1e3, roadAngle./pi*180); xlabel('km'); ylabel('deg'); title('slope');
%% create models:
csAllModels = CruiseParams(sSimParams.fs , gear, sSimParams.enableLinear);

%% create scenarios:
if newScenarios
    nModels = numel(csAllModels);
    modelsIdx = randperm(nModels,sSimParams.nScenarios);
    
    for scIdx = 1:sSimParams.nScenarios
        disp('starting cruise simulator')
        [y_fs,y_tVec,y,input_u,gearChange,sGroundTruth] = CruiseSimulator(sSimParams,csAllModels{modelsIdx(scIdx)},sInputs);
        
        %csSim{scIdx}.modelIdx = modelsIdx(scIdx);
        csSim{scIdx}.sModelParams = csAllModels{modelsIdx(scIdx)};
        csSim{scIdx}.sGroundTruth = sGroundTruth;
        csSim{scIdx}.input_u = input_u;
        csSim{scIdx}.y = y; csSim{scIdx}.y_tVec = y_tVec; csSim{scIdx}.y_fs = y_fs; csSim{scIdx}.gearChange = gearChange;
    end
    
    save('scenario.mat','csSim')
else
    load('scenario.mat');
end
%% figures
nScenarios = numel(csSim);
for scIdx = 1:nScenarios
    roadZ_atPos{scIdx} = interp1(roadX,roadZ,               csSim{scIdx}.sGroundTruth.pos,'spline');
    thetaDeg_atPos{scIdx} = asin(interp1(roadX,sin_theta,   csSim{scIdx}.sGroundTruth.pos,'spline'))/2/pi*360;
    v_kph{scIdx} = csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s;
end


figure;
for gearIdx = 1:5
    gearsIdx{gearIdx} = csSim{scIdx}.sGroundTruth.gears == gearIdx;
end
scIdx = 1;
ax(1) = subplot(4,1,1); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),v_kph{scIdx}(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    end
end
legend
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on;

ax(2) = subplot(4,1,2); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),roadZ_atPos{scIdx}(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(2)); xlabel('sec'); ylabel('m'); title('vertical position'); grid on;
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(2)); xlabel('sec'); grid on; ylabel('m'); title('vertical position');
    end
end
legend

ax(3) = subplot(4,1,3); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.u(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(3)); xlabel('sec'); title('u'); grid on;
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(3)); xlabel('sec'); grid on; title('u');
    end
end
legend

ax(4) = subplot(4,1,4); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.stateVec(2,gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(4)); xlabel('sec'); title('z'); ylabel('m'); grid on;
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(4)); xlabel('sec'); grid on; ylabel('m'); title('z');
    end
end
legend
linkaxes(ax,'x');

figure; hold all;
groundTruthTs = csSim{scIdx}.sGroundTruth.tVec(2)-csSim{scIdx}.sGroundTruth.tVec(1);
for scIdx = 1:nScenarios
    speedAtPos = ([0 ; diff(csSim{scIdx}.sGroundTruth.pos)]./groundTruthTs)./kph2m_s;
    plot(csSim{scIdx}.sGroundTruth.pos , speedAtPos);%,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]);
    grid on; xlabel('m'); ylabel('kph'); title('GroundTruth: speed vs pos');
end
%legend

figure;
subplot(2,1,1); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,roadZ_atPos{scIdx});%,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]);
    xlabel('m'); ylabel('m'); title('road vs car position'); grid on;
end
%legend

subplot(2,1,2);  hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,thetaDeg_atPos{scIdx});%,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]);
    xlabel('m'); ylabel('deg'); title('road slope vs car position'); grid on;
end
%legend

% figure;
% subplot(2,1,1);   hold all;
% for scIdx = 1:nScenarios
%     plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(1,:)./kph2m_s,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('sec'); ylabel('kph'); title('measured speed'); grid on;
% end
% legend
%
% subplot(2,1,2);   hold all;
% for scIdx = 1:nScenarios
%     plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(2,:),'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('sec'); ylabel('m'); title('measured cumulative error'); grid on;
% end
% legend

if nScenarios > 1
    disp('multiple scenarios is only for impression, kalman runs on single scenario');
    disp(['continue with scenario created from model: ',int2str(csSim{1}.modelIdx) ]);
    csSimTmp{1} = csSim{1};
    csSim = csSimTmp; clear csSimTmp;
end

figure;
subplot(2,1,1);   hold all;
scIdx = 1;
v_kph{scIdx} = csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s;
plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(1,:)./kph2m_s,'DisplayName',['measure']); xlabel('sec'); ylabel('kph'); title('speed'); grid on;

gearShiftUpIdx = find(csSim{scIdx}.gearChange == 1) - 1; gearShiftDownIdx = find(csSim{scIdx}.gearChange == -1) - 1;
stem(csSim{scIdx}.y_tVec(gearShiftUpIdx)    , 190*ones(size(csSim{scIdx}.y_tVec(gearShiftUpIdx))),'x','DisplayName',['gearShiftUp']);
stem(csSim{scIdx}.y_tVec(gearShiftDownIdx)  , 150*ones(size(csSim{scIdx}.y_tVec(gearShiftDownIdx))),'DisplayName',['gearShiftDown']);

plot(csSim{scIdx}.sGroundTruth.tVec,v_kph{scIdx},'LineWidth',2,'DisplayName','GroundTruth');
legend

display(['speed measurement std: ',num2str(std(csSim{scIdx}.y(1,:) - csSim{scIdx}.sGroundTruth.stateVec_atMeasureTimes(1,:))),' m/s']);
display(['controller measurement std: ',num2str(std(csSim{scIdx}.y(2,:) - csSim{scIdx}.sGroundTruth.stateVec_atMeasureTimes(2,:))),' m']);

subplot(2,1,2);   hold all;
plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(2,:),'DisplayName',['measure']); xlabel('sec'); ylabel('m'); title('cumulative error'); grid on;
plot(csSim{scIdx}.sGroundTruth.tVec,csSim{scIdx}.sGroundTruth.stateVec(2,:),'LineWidth',2,'DisplayName',['groundTruth']); xlabel('sec'); ylabel('m'); grid on;
legend

% write process statistics to screen:
speedDiffKph = diff(csSim{scIdx}.sGroundTruth.stateVec_atMeasureTimes(1,:)./kph2m_s); % [kph]
uDiff = diff(csSim{scIdx}.sGroundTruth.stateVec_atMeasureTimes(2,:)); % [m]
%display(['speed statistics: meanDiff: ',num2str(mean(speedDiffKph)),'; std: ',num2str(std(speedDiffKph)),' [kph]']);
%display(['controller statistics: meanDiff: ',num2str(mean(uDiff)),'; std: ',num2str(std(uDiff)),' [m]']);
%return
% look at the difference between the unmodeled behaviour of different
% gears:
nTimesSteps = size(csSim{scIdx}.sGroundTruth.stateVec_atMeasureTimes,2);
nSamplesToEvaluate = sSimParams.fs / sSimParams.desired_ySampleRate - 1;
sModelParams = csSim{1}.sModelParams;
tVecMat = zeros(nSamplesToEvaluate+1 , nTimesSteps); 
fullSpeedBehaviourAtCorrectGear = zeros(size(tVecMat)); fullControllBehaviourAtCorrectGear = zeros(size(fullSpeedBehaviourAtCorrectGear));
fullSpeedBehaviourAtAllGears = zeros(nSamplesToEvaluate+1 , nTimesSteps , 5); fullControllBehaviourAtAllGears = zeros(size(fullSpeedBehaviourAtAllGears));
partialSpeedBehaviourAtAllGears = zeros(size(fullSpeedBehaviourAtAllGears)); partialControllBehaviourAtAllGears = zeros(size(partialSpeedBehaviourAtAllGears));
for t = 1:nTimesSteps
    currentTime     = csSim{scIdx}.y_tVec(t);
    currentGear     = csSim{scIdx}.sGroundTruth.gears_atMeasureTimes(t);
    currentStateVec = csSim{scIdx}.sGroundTruth.stateVec_atMeasureTimes(:,t);
    currentInput_u  = csSim{scIdx}.input_u(:,t);
    
    tVecMat(1:1+nSamplesToEvaluate,t) = currentTime + transpose([0:nSamplesToEvaluate]./sSimParams.fs);
    
    % full behaviour at correct gear:
    fullSpeedBehaviourAtCorrectGear(1,t) = currentStateVec(1); fullControllBehaviourAtCorrectGear(1,t) = currentStateVec(2);
    nextStateVec = currentStateVec;
    for i = 1:nSamplesToEvaluate
        [nextStateVec,~] = CruiseTimeStep(nextStateVec, currentInput_u, sModelParams, sSimParams, currentGear, 0, false);
        fullSpeedBehaviourAtCorrectGear(i+1,t) = nextStateVec(1); fullControllBehaviourAtCorrectGear(i+1,t) = nextStateVec(2);
    end
    
    % full behaviour at all gears:
    for gearIdx = 1:5
        fullSpeedBehaviourAtAllGears(1,t,gearIdx) = currentStateVec(1); fullControllBehaviourAtAllGears(1,t,gearIdx) = currentStateVec(2);
        nextStateVec = currentStateVec;
        for i = 1:nSamplesToEvaluate
            [nextStateVec,~] = CruiseTimeStep(nextStateVec, currentInput_u, sModelParams, sSimParams, gearIdx, 0, false);
            fullSpeedBehaviourAtAllGears(i+1,t,gearIdx) = nextStateVec(1); fullControllBehaviourAtAllGears(i+1,t,gearIdx) = nextStateVec(2);
        end
    end     
    
    % partial behaviour at all gears:
    sModelParamsPartialBehaviour = sModelParams; sModelParamsPartialBehaviour.beta = 0;
    for gearIdx = 1:5
        partialSpeedBehaviourAtAllGears(1,t,gearIdx) = currentStateVec(1); partialControllBehaviourAtAllGears(1,t,gearIdx) = currentStateVec(2);
        nextStateVec = currentStateVec;
        for i = 1:nSamplesToEvaluate
            [nextStateVec,~] = CruiseTimeStep(nextStateVec, currentInput_u, sModelParamsPartialBehaviour, sSimParams, gearIdx, 0, false);
            partialSpeedBehaviourAtAllGears(i+1,t,gearIdx) = nextStateVec(1); partialControllBehaviourAtAllGears(i+1,t,gearIdx) = nextStateVec(2);
        end
    end       
end

partialSpeedBehaviourDiff = fullSpeedBehaviourAtAllGears - partialSpeedBehaviourAtAllGears;
partialControllBehaviourDiff = fullControllBehaviourAtAllGears - partialControllBehaviourAtAllGears;


figure; 

for gearIdx = 1:5
    gearsIdx{gearIdx} = csSim{scIdx}.sGroundTruth.gears == gearIdx;
end
scIdx = 1;
dh(1) = subplot(4,1,1); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),v_kph{scIdx}(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',dh(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',dh(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    end
end
legend

dh(2) = subplot(4,1,2); hold all; xlabel('sec'); ylabel('kph'); title('speed diff between full and partial behaviour');
dh(3) = subplot(4,1,3); hold all; xlabel('sec'); ylabel('m');   title('controll diff between full and partial behaviour');
for t = 1:nTimesSteps
    subplot(4,1,2); hold all;     
    plot(tVecMat(:,t) , partialSpeedBehaviourDiff(:,t,1)./kph2m_s,'b', 'Parent',dh(2),'DisplayName',['partial gear ',int2str(1)]);
    plot(tVecMat(:,t) , partialSpeedBehaviourDiff(:,t,2)./kph2m_s,'r', 'Parent',dh(2),'DisplayName',['partial gear ',int2str(2)]);
    plot(tVecMat(:,t) , partialSpeedBehaviourDiff(:,t,3)./kph2m_s,'y', 'Parent',dh(2),'DisplayName',['partial gear ',int2str(3)]);
    plot(tVecMat(:,t) , partialSpeedBehaviourDiff(:,t,4)./kph2m_s,'m', 'Parent',dh(2),'DisplayName',['partial gear ',int2str(4)]);
    plot(tVecMat(:,t) , partialSpeedBehaviourDiff(:,t,5)./kph2m_s,'g', 'Parent',dh(2),'DisplayName',['partial gear ',int2str(5)]);
%     for gearIdx = 1:5
%         plot(tVecMat(:,t) , partialSpeedBehaviourAtAllGears(:,t,gearIdx), 'DisplayName',['partial gear ',int2str(gearIdx)]);
%     end
    subplot(4,1,3); hold all;     
    plot(tVecMat(:,t) , partialControllBehaviourDiff(:,t,1),'b', 'Parent',dh(3),'DisplayName',['partial gear ',int2str(1)]);
    plot(tVecMat(:,t) , partialControllBehaviourDiff(:,t,2),'r', 'Parent',dh(3),'DisplayName',['partial gear ',int2str(2)]);
    plot(tVecMat(:,t) , partialControllBehaviourDiff(:,t,3),'y', 'Parent',dh(3),'DisplayName',['partial gear ',int2str(3)]);
    plot(tVecMat(:,t) , partialControllBehaviourDiff(:,t,4),'m', 'Parent',dh(3),'DisplayName',['partial gear ',int2str(4)]);
    plot(tVecMat(:,t) , partialControllBehaviourDiff(:,t,5),'g', 'Parent',dh(3),'DisplayName',['partial gear ',int2str(5)]);
%     plot(tVecMat(:,t) , fullControllBehaviourAtCorrectGear(:,t), 'DisplayName',['fullCorrect']);
%     for gearIdx = 1:5
%         plot(tVecMat(:,t) , partialControllBehaviourAtAllGears(:,t,gearIdx), 'DisplayName',['partial gear ',int2str(gearIdx)]);
%     end
end
subplot(4,1,2); legend({'partia gear 1','partia gear 2','partia gear 3','partia gear 4','partia gear 5'})
subplot(4,1,3); legend({'partia gear 1','partia gear 2','partia gear 3','partia gear 4','partia gear 5'})
dh(4) = subplot(4,1,4); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.u(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',dh(4)); xlabel('sec'); title('u'); grid on;
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',dh(4)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    end
end
legend

linkaxes(dh,'x');
%return
%% create kalman matrices for every model:
y_fs            = csSim{1}.y_fs;
nModels         = numel(csAllModels);
uInput_init     = csSim{1}.input_u(:,1);

for modelIdx = 1:nModels
    sKalmanMatrices{modelIdx} = CruiseKalmanParams(csAllModels{modelIdx}, y_fs, sSimParams.fs);
    % for vRef = 80kph stable u (throttle level) was measured to be 0.5491 when
    % Ki was 0.0015; therefore stable z is 0.5491/0.0015 = 366.0718
    sInitValues{modelIdx}.xPlusMean_init      = [vNominal_kph * kph2m_s; 366.0781];
    sInitValues{modelIdx}.xPlusCov_init       = [(40*kph2m_s)^2 , 0 ; 0 , 500^2];
    sInitValues{modelIdx}.uInput_init         = uInput_init;
    sInitValues{modelIdx}.weight              = 1/nModels;
    sInitValues{modelIdx}.logWeight           = -log(nModels);
    sKalmanMatrices{modelIdx}.modelIdx        = modelIdx;
end
%% run inference:
% my derivation:
if sSimParams.doMyFiltering
    csKalmanRes = InferenceScheme(csSim{1} , sKalmanMatrices , sInitValues , sSimParams , csAllModels);
end


% Barber Filtering:
I = 1;

y = csSim{1}.y; u = csSim{1}.input_u;
xInitMean = sInitValues{3}.xPlusMean_init; xInitCov = sInitValues{3}.xPlusCov_init; uInit = sInitValues{3}.uInput_init;
tranS = transpose(csAllModels{1}.transitionMat);
switchTimeIndexes = find(csSim{1}.gearChange ~= 0); % switchTimeIndexes ==1 @(i) indicates that a new gear operated at (i-1)

% due to decimation:
%switchTimeIndexes = sort([switchTimeIndexes ; switchTimeIndexes + 1]);

S = numel(sKalmanMatrices);
for s=1:S
    F(:,:,s) = sKalmanMatrices{s}.F; G(:,:,s) = sKalmanMatrices{s}.G; H(:,:,s) = sKalmanMatrices{s}.C; Q(:,:,s) = sKalmanMatrices{s}.Q; R(:,:,s) = sKalmanMatrices{s}.R;
    priorS(s,1) = sInitValues{s}.weight;
end

% %S = numel(sKalmanMatrices);
% %for s=1:S
% gearIdx = 3; s = 1; S = 1;
%     F(:,:,s) = sKalmanMatrices{gearIdx}.F; G(:,:,s) = sKalmanMatrices{gearIdx}.G; H(:,:,s) = sKalmanMatrices{gearIdx}.C; Q(:,:,s) = sKalmanMatrices{gearIdx}.Q; R(:,:,s) = sKalmanMatrices{gearIdx}.R;
%     priorS(s,1) = sInitValues{gearIdx}.weight;
% %end

display(['kalman input speed meas std: ', num2str(sqrt(R(1,1,1))),' m/s']);
display(['kalman input controller meas std: ', num2str(sqrt(R(2,2,1))),' m']);

for t = 1:(size(csSim{1}.sGroundTruth.stateVec_atMeasureTimes,2)-1)
    currState = csSim{1}.sGroundTruth.stateVec_atMeasureTimes(:,t);
    currGear = csSim{1}.sGroundTruth.gears_atMeasureTimes(t);
    currF = F(:,:,currGear);
    currG = G(:,:,currGear);
    currU = [u(:,t) ; sign(currState(1))];
    expectedState(:,t+1) = currF*currState + currG*currU;
end

processNoise = csSim{1}.sGroundTruth.stateVec_atMeasureTimes - expectedState;

display(['groundTruth speed process std: ',num2str(std(processNoise(1,:))),' m/s']);
display(['groundTruth controller process std: ',num2str(std(processNoise(2,:))),' m']);

if sqrt(Q(1,1,1)) > std(processNoise(1,:))
    display(['kalman input speed process std: ', num2str(sqrt(Q(1,1,1))),' m/s']);
else
    display(['kalman input speed process std: ', num2str(sqrt(Q(1,1,1))),' m/s   !!!!!!!!!!!!!!!!!!!!!!!!!!']);
end

if sqrt(Q(2,2,1)) > std(processNoise(2,:))
    display(['kalman input controller process std: ', num2str(sqrt(Q(2,2,1))),' m']);
else
    display(['kalman input controller process std: ', num2str(sqrt(Q(2,2,1))),' m   !!!!!!!!!!!!!!!!!!!!!!!!!!']);
end

enableCruiseCustom = true;
[xEstfMean, xEstfCov, xEstfMinus_mean, xEstfMinus_cov, alpha, w, loglik] = SLDSforward(y,u,switchTimeIndexes,F,G,H,Q,R,xInitCov,xInitMean,uInit,tranS,priorS,I,enableCruiseCustom);

% smoothing:
doEC = true; J = 1;
[xEstMean, xEstCov, gamma, u] = SLDSbackward(switchTimeIndexes, xEstfMean, xEstfCov, xEstfMinus_mean, xEstfMinus_cov, alpha, w, F, Q, tranS, I, J, doEC);
%% ANALYZE
if sSimParams.doMyFiltering
    % my inference:
    figure;
    scIdx = 1;
    
    for gearIdx = 1:5
        gearsIdx{gearIdx} = csSim{scIdx}.sGroundTruth.gears == gearIdx;
    end
    scIdx = 1;
    ax(1) = subplot(2,1,1); hold all;
    for gearIdx = 1:5
        if any(gearsIdx{gearIdx})
            plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.stateVec(1,gearsIdx{gearIdx})./kph2m_s,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
        else
            plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
        end
    end
    legend
    
  
    ax(2) = subplot(2,1,2); hold all;
    for filteringIdx = 1:numel(csKalmanRes)
        assert(numel(unique(csKalmanRes{filteringIdx}.kalmanModelIdx))==1)
        plot(csKalmanRes{filteringIdx}.tVec , 20*log10(csKalmanRes{filteringIdx}.weight),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx(1))]);
    end
    xlabel('sec'); ylabel('db'); title(['weights']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
    ylim([-30 0]);
    legend
    
    linkaxes(ax,'x');
    
    figure;
    scIdx = 1;
    
    for gearIdx = 1:5
        gearsIdx{gearIdx} = csSim{scIdx}.sGroundTruth.gears == gearIdx;
    end
    scIdx = 1;
    ax(3) = subplot(2,1,1); hold all;
    for gearIdx = 1:5
        if any(gearsIdx{gearIdx})
            plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.stateVec(1,gearsIdx{gearIdx})./kph2m_s,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(3)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
        else
            plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(3)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
        end
    end
    legend
    
    
    ax(4) = subplot(2,1,2); hold all;
    for filteringIdx = 1:numel(csKalmanRes)
        assert(numel(unique(csKalmanRes{filteringIdx}.kalmanModelIdx))==1)
        log10Weight = (csKalmanRes{filteringIdx}.logWeight) ./ log(10);
        plot(csKalmanRes{filteringIdx}.tVec , 20*log10Weight,'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx(1))],'Parent',ax(4));
    end
    xlabel('sec'); ylabel('db'); title(['weights (logPropagation)']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
    ylim([-30 0]);
    legend
    
    linkaxes(ax,'x');
    
    
    
    % subplot(3,1,3); hold all;
    % for filteringIdx = 1:numel(csKalmanRes)
    %     plot(csKalmanRes{filteringIdx}.tVec , -20*log10(csKalmanRes{filteringIdx}.xPlusCovTrace),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx)]);
    % end
    % xlabel('sec'); ylabel('db'); title(['1/trace(cov)']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
    % legend
    
    % figure; hold all;
    % for filteringIdx = 1:numel(csKalmanRes)
    %     plot(csKalmanRes{filteringIdx}.tVec , -20*log10(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:))),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx)]);
    % end
    % xlabel('sec'); ylabel('db'); title(['1/speedVar']);%; true sc model: ',int2str(csSim{1}.modelIdx)]);
    % legend
    
    
    
    %     for filteringIdx = 1:numel(csKalmanRes)
    %         %scModelIdx      = csKalmanRes{filteringIdx}.scenarioModelIdx;
    %         kalmanModelIdx  = csKalmanRes{filteringIdx}.kalmanModelIdx;
    %         scIdx           = 1;%csKalmanRes{filteringIdx}.scenarioIdx;
    %
    %
    %         figure;
    %         subplot(2,1,1); hold all;
    %         errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:)))./kph2m_s );
    %         plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    %         title(['scModel: ',int2str(filteringIdx),'; filtered speed']); legend('filtered','true');
    %
    %         subplot(2,1,2); hold all;
    %         errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(2,:) , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(2,2,:))));
    %         plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
    %         title('filtered control-z'); legend('filtered','true');
    %
    %     end
    
    for filteringIdx = 1:numel(csKalmanRes)
        myAlpha(filteringIdx,:) = csKalmanRes{filteringIdx}.weight;
    end
    
    for filteringIdx = 1:numel(csKalmanRes)
        for i=1:numel(csSim{scIdx}.y_tVec)
            myxPlusMean(:,filteringIdx,i) = csKalmanRes{filteringIdx}.xPlusMean(:,i);
            myxPlusCov(:,:,filteringIdx,i) = csKalmanRes{filteringIdx}.xPlusCov(:,:,i);
        end
    end
    
    [~,sMax] = max(myAlpha);
    figure;
    for i=1:numel(csSim{scIdx}.y_tVec)
        xPlusMeanSmax(i) = myxPlusMean(1,sMax(i),i);
        xPlusCovSmax(i) = myxPlusCov(1,1,sMax(i),i);
    end
    
    
    subplot(2,1,1); hold all;
    errorbar(csSim{scIdx}.y_tVec , xPlusMeanSmax./kph2m_s , 3*sqrt(xPlusCovSmax)./kph2m_s );
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    title(['my filtered speed']); legend('filtered','true');
    
    for i=1:numel(csSim{scIdx}.y_tVec)
        xPlusMeanSmax(i) = myxPlusMean(2,sMax(i),i);
        xPlusCovSmax(i) = myxPlusCov(2,2,sMax(i),i);
    end
    
    subplot(2,1,2); hold all;
    errorbar(csSim{scIdx}.y_tVec , xPlusMeanSmax./kph2m_s , 3*sqrt(xPlusCovSmax)./kph2m_s );
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
    title('my filtered control-z'); legend('filtered','true');
    
end
% barbel:
figure;
scIdx = 1;

for gearIdx = 1:5
    gearsIdx{gearIdx} = csSim{scIdx}.sGroundTruth.gears == gearIdx;
end
scIdx = 1;

bx(1) = subplot(4,1,1); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.stateVec(1,gearsIdx{gearIdx})./kph2m_s,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',bx(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',bx(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
    end
end
legend

bx(2) = subplot(4,1,2); hold all;
for gearIdx = 1:5
    if any(gearsIdx{gearIdx})
        plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.u(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',bx(2)); xlabel('sec'); title('u'); grid on;
    else
        plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',bx(2)); xlabel('sec'); grid on; ylabel('kph'); 
    end
end

bx(3) = subplot(4,1,3); hold all;
[~,sMax] = max(alpha);
correctFilteredIndexes = (sMax' == csSim{scIdx}.sGroundTruth.gears_atMeasureTimes);
for filteringIdx = 1:S
    plot(csSim{1}.y_tVec , alpha(filteringIdx,:),'.-','DisplayName',['kModel: ',int2str(filteringIdx)],'Parent',bx(3));
end
j = 0; clear errorVec;
for i=1:numel(csSim{1}.y_tVec)
    if not(correctFilteredIndexes(i))
        %plot(csSim{1}.y_tVec(i) , alpha(sMax(i),i),'*k');
        j = j + 1;
        errorVec(:,j) = [csSim{1}.y_tVec(i) ; alpha(sMax(i),i)];
    end
end
plot(errorVec(1,:) , errorVec(2,:),'.k','DisplayName',['errors'],'Parent',bx(3));
xlabel('sec'); title(['filtered weights by barber; ',int2str(length(errorVec)),' errors']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
ylim([0 1]);
legend

%linkaxes(bx,'x');

%figure;
scIdx = 1;

% for gearIdx = 1:5
%     gearsIdx{gearIdx} = csSim{scIdx}.sGroundTruth.gears == gearIdx;
% end
% scIdx = 1;
% bx(3) = subplot(2,1,1); hold all;
% for gearIdx = 1:5
%     if any(gearsIdx{gearIdx})
%         plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.stateVec(1,gearsIdx{gearIdx})./kph2m_s,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',bx(3)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
%     else
%         plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',bx(3)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
%     end
% end
% legend


bx(4) = subplot(4,1,4); hold all;
[~,sMax] = max(gamma);
correctFilteredIndexes = (sMax' == csSim{scIdx}.sGroundTruth.gears_atMeasureTimes);
for filteringIdx = 1:S
    
    plot(csSim{1}.y_tVec , gamma(filteringIdx,:),'.-','DisplayName',['kModel: ',int2str(filteringIdx)],'Parent',bx(4));
end
j = 0; clear errorVec;
for i=1:numel(csSim{1}.y_tVec)
    if not(correctFilteredIndexes(i))
        %plot(csSim{1}.y_tVec(i) , alpha(sMax(i),i),'*k');
        j = j + 1;
        errorVec(:,j) = [csSim{1}.y_tVec(i) ; gamma(sMax(i),i)];
    end
end
plot(errorVec(1,:) , errorVec(2,:),'.k','DisplayName',['errors'],'Parent',bx(4));
xlabel('sec'); title(['smoothed weights by barber; ',int2str(length(errorVec)),' errors']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
ylim([0 1]);
legend

linkaxes(bx,'x');

figure;
[~,sMax] = max(alpha);

for i=1:size(xEstMean,4)
    xEstMeanSmax(i) =  xEstfMean(1,1,sMax(i),i);
    xEstCovSmax(i) = xEstfCov(1,1,1,sMax(i),i);
end

subplot(2,1,1); hold all;
errorbar(csSim{scIdx}.y_tVec , xEstMeanSmax./kph2m_s , 3*sqrt(xEstCovSmax)./kph2m_s );
plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
title(['filtered speed']); legend('filtered','true');

for i=1:size(xEstMean,4)
    xEstMeanSmax(i) =  xEstfMean(2,1,sMax(i),i);
    xEstCovSmax(i) = xEstfCov(2,2,1,sMax(i),i);
end

subplot(2,1,2); hold all;
errorbar(csSim{scIdx}.y_tVec , xEstMeanSmax , 3*sqrt(xEstCovSmax) );
plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
title('filtered control-z'); legend('filtered','true');


figure;
[~,sMax] = max(gamma);

for i=1:size(xEstMean,4)
    xEstMeanSmax(i) =  xEstMean(1,1,sMax(i),i);
    xEstCovSmax(i) = xEstCov(1,1,1,sMax(i),i);
end

subplot(2,1,1); hold all;
errorbar(csSim{scIdx}.y_tVec , xEstMeanSmax./kph2m_s , 3*sqrt(xEstCovSmax)./kph2m_s );
plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
title(['smoothed speed']); legend('smoothed','true');

for i=1:size(xEstMean,4)
    xEstMeanSmax(i) =  xEstMean(2,1,sMax(i),i);
    xEstCovSmax(i) = xEstCov(2,2,1,sMax(i),i);
end

subplot(2,1,2); hold all;
errorbar(csSim{scIdx}.y_tVec , xEstMeanSmax , 3*sqrt(xEstCovSmax) );
plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
title('smoothed control-z'); legend('smoothed','true');