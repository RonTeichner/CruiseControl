clear; close all; clc;
newRoad = true;
newScenarios = true;

vNominal_kph = 80; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]

sSimParams.nSeed = 697650481;
sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 30*60; % [sec]
sSimParams.simDistance = max(10e3, sSimParams.simDuration * (2*vNominal)); % [m]
sSimParams.enableLinear = true;
sSimParams.nScenarios = 1;
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
%% create models:
csAllModels = CruiseParams(sSimParams.fs , gear, sSimParams.enableLinear);

%% create scenarios:
if newScenarios
    nModels = numel(csAllModels);
    modelsIdx = randperm(nModels,sSimParams.nScenarios);
    
    for scIdx = 1:sSimParams.nScenarios
        disp('starting cruise simulator')
        [y_fs,y_tVec,y,input_u,sGroundTruth] = CruiseSimulator(sSimParams,csAllModels{modelsIdx(scIdx)},sInputs);
        
        csSim{scIdx}.modelIdx = modelsIdx(scIdx);
        csSim{scIdx}.sModelParams = csAllModels{modelsIdx(scIdx)};
        csSim{scIdx}.sGroundTruth = sGroundTruth;
        csSim{scIdx}.input_u = input_u;
        csSim{scIdx}.y = y; csSim{scIdx}.y_tVec = y_tVec; csSim{scIdx}.y_fs = y_fs;
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

subplot(4,1,1); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,v_kph{scIdx},'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
end
legend
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on;

subplot(4,1,2); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,roadZ_atPos{scIdx},'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('sec'); ylabel('m'); title('vertical position'); grid on;
end
legend

subplot(4,1,3); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,csSim{scIdx}.sGroundTruth.u,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('sec'); title('u'); grid on;
end
legend

subplot(4,1,4); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.tVec,csSim{scIdx}.sGroundTruth.stateVec(2,:),'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('sec'); title('z'); ylabel('m'); grid on;
end
legend

figure; hold all;
groundTruthTs = csSim{scIdx}.sGroundTruth.tVec(2)-csSim{scIdx}.sGroundTruth.tVec(1);
for scIdx = 1:nScenarios
    speedAtPos = ([0 ; diff(csSim{scIdx}.sGroundTruth.pos)]./groundTruthTs)./kph2m_s;
    plot(csSim{scIdx}.sGroundTruth.pos , speedAtPos,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); grid on; xlabel('m'); ylabel('kph'); title('GroundTruth: speed vs pos');
end
legend

figure;
subplot(2,1,1); hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,roadZ_atPos{scIdx},'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('m'); ylabel('m'); title('road'); grid on;
end
legend

subplot(2,1,2);  hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,thetaDeg_atPos{scIdx},'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]); xlabel('m'); ylabel('deg'); title('road slope'); grid on;
end
legend

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
plot(csSim{scIdx}.sGroundTruth.tVec,v_kph{scIdx},'LineWidth',2,'DisplayName','GroundTruth');
legend

subplot(2,1,2);   hold all;
plot(csSim{scIdx}.y_tVec , csSim{scIdx}.y(2,:),'DisplayName',['measure']); xlabel('sec'); ylabel('m'); title('cumulative error'); grid on;
plot(csSim{scIdx}.sGroundTruth.tVec,csSim{scIdx}.sGroundTruth.stateVec(2,:),'LineWidth',2,'DisplayName',['groundTruth']); xlabel('sec'); ylabel('m'); grid on;
legend
%% create kalman matrices for every model:
y_fs            = csSim{1}.y_fs;
nModels         = numel(csAllModels);
uInput_init     = csSim{1}.input_u(:,1);

for modelIdx = 1:nModels
    sKalmanMatrices{modelIdx} = CruiseKalmanParams(csAllModels{modelIdx}, y_fs, sSimParams.fs);
    % for vRef = 80kph stable u (throttle level) was measured to be 0.5491 when
    % Ki was 0.0015; therefore stable z is 0.5491/0.0015 = 366.0718
    sInitValues{modelIdx}.xPlusMean_init      = [vNominal_kph * kph2m_s; 366.0781];
    sInitValues{modelIdx}.xPlusCov_init       = [(5*kph2m_s)^2 , 0 ; 0 , 50^2];
    sInitValues{modelIdx}.uInput_init         = uInput_init;
    sKalmanMatrices{modelIdx}.modelIdx        = modelIdx;
end
%% run for every combination of model and scenario:
filteringIdx = 0;
nModels = numel(sKalmanMatrices);
nScenarios = 1;
for scIdx = 1:nScenarios
    for modelIdx = 1:nModels
        [xPlusMean , xPlusCov , weightFactor] = CruiseKalmanFilter(sKalmanMatrices{modelIdx},sInitValues{modelIdx}, csSim{scIdx}.y, csSim{scIdx}.input_u);
        
        filteringIdx = filteringIdx + 1;
        
        csKalmanRes{filteringIdx}.scenarioModelIdx  = csSim{scIdx}.modelIdx;
        csKalmanRes{filteringIdx}.kalmanModelIdx    = sKalmanMatrices{modelIdx}.modelIdx;
        csKalmanRes{filteringIdx}.sKalmanMatrices   = sKalmanMatrices{modelIdx};
        csKalmanRes{filteringIdx}.sInitValues       = sInitValues{modelIdx};
        csKalmanRes{filteringIdx}.scenarioIdx       = scIdx;
        csKalmanRes{filteringIdx}.tVec              = csSim{scIdx}.y_tVec;
        csKalmanRes{filteringIdx}.xPlusMean = xPlusMean; csKalmanRes{filteringIdx}.xPlusCov = xPlusCov; csKalmanRes{filteringIdx}.weightFactor = weightFactor;
        for timeIdx = 1:size(xPlusCov,3)
            csKalmanRes{filteringIdx}.xPlusCovTrace(timeIdx) = trace(xPlusCov(:,:,timeIdx));
        end
    end
end

% calculate weight after every time-step:
nTimeSteps  = numel(csSim{1}.y(1,:));
nFilters    = numel(csKalmanRes);
for timeIdx = 1:nTimeSteps
    for filteringIdx = 1:nFilters
        weightFactors(filteringIdx) = csKalmanRes{filteringIdx}.weightFactor(timeIdx);
    end
    
    % not necessary but convenient to normaliaze weight factors:
    weightFactors = weightFactors ./ sum(weightFactors);
    
    if timeIdx > 1
        for filteringIdx = 1:nFilters
            previousWeights(filteringIdx) = csKalmanRes{filteringIdx}.weight(timeIdx - 1);
        end
        
        newWeights = previousWeights .* weightFactors;
        newWeights = newWeights ./ sum(newWeights);
    else
        newWeights = weightFactors ./ sum(weightFactors);
    end
    
    for filteringIdx = 1:nFilters
        csKalmanRes{filteringIdx}.weight(timeIdx) = newWeights(filteringIdx);
    end
    
end
%% ANALYZE

figure;
scIdx = 1;
subplot(3,1,1); hold all;
plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
subplot(3,1,2); hold all;
for filteringIdx = 1:numel(csKalmanRes)
    plot(csKalmanRes{filteringIdx}.tVec , 20*log10(csKalmanRes{filteringIdx}.weight),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx)]);
end
xlabel('sec'); ylabel('db'); title(['weights; true sc model: ',int2str(csSim{1}.modelIdx)]);
legend

subplot(3,1,3); hold all;
for filteringIdx = 1:numel(csKalmanRes)
    plot(csKalmanRes{filteringIdx}.tVec , -20*log10(csKalmanRes{filteringIdx}.xPlusCovTrace),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx)]);
end
xlabel('sec'); ylabel('db'); title(['1/trace(cov); true sc model: ',int2str(csSim{1}.modelIdx)]);
legend

% figure; 
% for filteringIdx = 1:numel(csKalmanRes)
%     subplot(numel(csKalmanRes),1,filteringIdx);
%     plot(csKalmanRes{filteringIdx}.tVec , 20*log10(csKalmanRes{filteringIdx}.weight),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx)]); xlabel('sec'); ylabel('db'); legend;
% end

for filteringIdx = 1:numel(csKalmanRes)
    scModelIdx      = csKalmanRes{filteringIdx}.scenarioModelIdx;
    kalmanModelIdx  = csKalmanRes{filteringIdx}.kalmanModelIdx;
    scIdx           = csKalmanRes{filteringIdx}.scenarioIdx;
    
    
    figure;
    subplot(2,1,1); hold all;
    errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:)))./kph2m_s );
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    title(['scModel: ',int2str(scModelIdx),'; kModel: ',int2str(kalmanModelIdx),'; filtered speed']); legend('filtered','true');
    
    subplot(2,1,2); hold all;
    errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(2,:) , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(2,2,:))));
    plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
    title('filtered control-z'); legend('filtered','true');
    
    %     figure;
    %     firstSamleToPlot = 100;
    %     csKalmanRes{filteringIdx}.xPlusCovTrace = zeros(size(csKalmanRes{filteringIdx}.xPlusCov,3),1);
    %     for i=1:size(csKalmanRes{filteringIdx}.xPlusCov,3)
    %         csKalmanRes{filteringIdx}.xPlusCovTrace(i) = trace(csKalmanRes{filteringIdx}.xPlusCov(:,:,i));
    %     end
    %     subplot(3,1,1);
    %     plot(csSim{scIdx}.y_tVec(firstSamleToPlot:end) , csKalmanRes{filteringIdx}.xPlusCovTrace(firstSamleToPlot:end)); xlabel('sec'); title(['scModel: ',int2str(scModelIdx),'; kModel: ',int2str(kalmanModelIdx),'; trace of covMat']); grid on;
    %     subplot(3,1,2);
    %     plot(csSim{scIdx}.y_tVec(firstSamleToPlot:end) , csKalmanRes{filteringIdx}.weightFactor(firstSamleToPlot:end)); xlabel('sec'); title('weight factor'); grid on;
    %     subplot(3,1,3);hold all;
    %     errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:)))./kph2m_s );
    %     plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    %     title('filtered speed'); legend('filtered','true');
end
