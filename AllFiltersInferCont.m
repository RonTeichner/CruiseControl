function csKalmanRes = AllFiltersInferCont(sKalmanMatrices, sInitValues , sScenario)
filteringIdx = 0;
nModels = numel(sKalmanMatrices);


for modelIdx = 1:nModels
    [xPlusMean , xPlusCov , weightFactor , xMinusMean , xMinusCov] = CruiseKalmanFilter(sKalmanMatrices{modelIdx},sInitValues{modelIdx}, sScenario.y, sScenario.input_u);
    
    filteringIdx = filteringIdx + 1;
    
    %csKalmanRes{filteringIdx}.scenarioModelIdx  = sScenario.modelIdx;
    csKalmanRes{filteringIdx}.kalmanModelIdx    = sKalmanMatrices{modelIdx}.modelIdx;
    csKalmanRes{filteringIdx}.sKalmanMatrices   = sKalmanMatrices{modelIdx};
    csKalmanRes{filteringIdx}.sInitValues       = sInitValues{modelIdx};
    %csKalmanRes{filteringIdx}.scenarioIdx       = 1;
    csKalmanRes{filteringIdx}.tVec              = sScenario.y_tVec;
    csKalmanRes{filteringIdx}.xPlusMean = xPlusMean; csKalmanRes{filteringIdx}.xPlusCov = xPlusCov; csKalmanRes{filteringIdx}.weightFactor = weightFactor;
    csKalmanRes{filteringIdx}.xMinusMean = xMinusMean; csKalmanRes{filteringIdx}.xMinusCov = xMinusCov;
    for timeIdx = 1:size(xPlusCov,3)
        csKalmanRes{filteringIdx}.xPlusCovTrace(timeIdx) = trace(xPlusCov(:,:,timeIdx));
    end
end

%% calculate weight after every time-step:
for modelIdx = 1:nModels
    previousWeights(modelIdx) = sInitValues{modelIdx}.weight;
end

nTimeSteps  = numel(sScenario.y(1,:));
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
    end
    newWeights = previousWeights .* weightFactors;
    newWeights = newWeights ./ sum(newWeights);
    
    
    for filteringIdx = 1:nFilters
        csKalmanRes{filteringIdx}.weight(timeIdx) = newWeights(filteringIdx);
    end
    
end
