clear; close all; clc;
newRoad = false;
newScenarios = false;

vNominal_kph = 60; % [kph]
kph2m_s = 1000/60/60;
vNominal = vNominal_kph*kph2m_s; % [m/s]

sSimParams.nSeed = 7707660;%round(1e7*rand);
sSimParams.fs = 100; % [hz]
sSimParams.simDuration = 30*60; % [sec]
sSimParams.simDistance = max(10e3, sSimParams.simDuration * (2*vNominal)); % [m]
sSimParams.enableLinear = true;
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
    if ~any(gearsIdx{1})
        gearsIdx{gearIdx}(1) = 1;
    end
end
scIdx = 1;
ax(1) = subplot(4,1,1); hold all;
for gearIdx = 1:5
    plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),v_kph{scIdx}(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
end
legend
%subplot(4,1,2); plot(pos,theta/(2*pi)*360); xlabel('m'); ylabel('angle'); grid on;

ax(2) = subplot(4,1,2); hold all;
for gearIdx = 1:5
    plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),roadZ_atPos{scIdx}(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(2)); xlabel('sec'); ylabel('m'); title('vertical position'); grid on;
end
legend

ax(3) = subplot(4,1,3); hold all;
for gearIdx = 1:5
    plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.u(gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(3)); xlabel('sec'); title('u'); grid on;
end
legend

ax(4) = subplot(4,1,4); hold all;
for gearIdx = 1:5
    plot(csSim{scIdx}.sGroundTruth.tVec(gearsIdx{gearIdx}),csSim{scIdx}.sGroundTruth.stateVec(2,gearsIdx{gearIdx}),'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(4)); xlabel('sec'); title('z'); ylabel('m'); grid on;
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
    xlabel('m'); ylabel('m'); title('road'); grid on;
end
%legend

subplot(2,1,2);  hold all;
for scIdx = 1:nScenarios
    plot(csSim{scIdx}.sGroundTruth.pos,thetaDeg_atPos{scIdx});%,'DisplayName',['model: ',int2str(csSim{scIdx}.modelIdx)]);
    xlabel('m'); ylabel('deg'); title('road slope'); grid on;
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

gearShiftUpIdx = (csSim{scIdx}.gearChange == 1); gearShiftDownIdx = (csSim{scIdx}.gearChange == -1);
stem(csSim{scIdx}.y_tVec(gearShiftUpIdx)    , 190*ones(size(csSim{scIdx}.y_tVec(gearShiftUpIdx))),'x','DisplayName',['gearShiftUp']);
stem(csSim{scIdx}.y_tVec(gearShiftDownIdx)  , 150*ones(size(csSim{scIdx}.y_tVec(gearShiftDownIdx))),'DisplayName',['gearShiftDown']);

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
switchTimeIndexes = find(csSim{1}.gearChange ~= 0);

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

[xEstfMean, xEstfCov, xEstfMinus_mean, xEstfMinus_cov, alpha, w, loglik] = SLDSforward(y,u,switchTimeIndexes,F,G,H,Q,R,xInitCov,xInitMean,uInit,tranS,priorS,I);

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
        log10Weight = (csKalmanRes{filteringIdx}.logWeight) ./ log(10);
        plot(csKalmanRes{filteringIdx}.tVec , 20*log10Weight,'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx(1))]);
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
    
    
    
    % for filteringIdx = 1:numel(csKalmanRes)
    %     scModelIdx      = csKalmanRes{filteringIdx}.scenarioModelIdx;
    %     kalmanModelIdx  = csKalmanRes{filteringIdx}.kalmanModelIdx;
    %     scIdx           = 1;%csKalmanRes{filteringIdx}.scenarioIdx;
    %
    %
    %     figure;
    %     subplot(2,1,1); hold all;
    %     errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(1,:)./kph2m_s , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(1,1,:)))./kph2m_s );
    %     plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(1,:)./kph2m_s); xlabel('sec'); ylabel('kph'); grid on;
    %     title(['scModel: ',int2str(scModelIdx),'; kModel: ',int2str(kalmanModelIdx),'; filtered speed']); legend('filtered','true');
    %
    %     subplot(2,1,2); hold all;
    %     errorbar(csSim{scIdx}.y_tVec , csKalmanRes{filteringIdx}.xPlusMean(2,:) , 3*sqrt(squeeze(csKalmanRes{filteringIdx}.xPlusCov(2,2,:))));
    %     plot(csSim{scIdx}.sGroundTruth.tVec , csSim{scIdx}.sGroundTruth.stateVec(2,:)); xlabel('sec'); ylabel('m'); grid on;
    %     title('filtered control-z'); legend('filtered','true');
    %
    % end
end
% barbel:
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
for filteringIdx = 1:S
    
    plot(csSim{1}.y_tVec , alpha(filteringIdx,:),'.-','DisplayName',['kModel: ',int2str(filteringIdx)]);
end
xlabel('sec'); title(['filtered weights by barber']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
ylim([0 1]);
legend

linkaxes(ax,'x');

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
for filteringIdx = 1:S
    
    plot(csSim{1}.y_tVec , gamma(filteringIdx,:),'.-','DisplayName',['kModel: ',int2str(filteringIdx)]);
end
xlabel('sec'); title(['smoothed weights by barber']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
ylim([0 1]);
legend

linkaxes(ax,'x');

% debug:
% smoothing:
doEC = true; J = 1;
maxTime = 1680; % [sec];
maxTimeIdx = find(csSim{1}.y_tVec > maxTime,1,'first');

[xEstMean, xEstCov, gamma, u] = SLDSbackward(switchTimeIndexes, xEstfMean, xEstfCov, xEstfMinus_mean, xEstfMinus_cov, alpha, w, F, Q, tranS, I, J, doEC);