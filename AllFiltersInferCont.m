function csKalmanRes = AllFiltersInferCont(sKalmanMatrices, sInitValues , sScenario, enableNormalizeWeights)
filteringIdx = 0;
nModels = numel(sKalmanMatrices);


for modelIdx = 1:nModels
    [xPlusMean , xPlusCov , weightFactor , xMinusMean , xMinusCov , logWeightFactor] = CruiseKalmanFilter(sKalmanMatrices{modelIdx},sInitValues{modelIdx}, sScenario);
    
    filteringIdx = filteringIdx + 1;
    
    %csKalmanRes{filteringIdx}.scenarioModelIdx  = sScenario.modelIdx;
    csKalmanRes{filteringIdx}.kalmanModelIdx    = sKalmanMatrices{modelIdx}.modelIdx;
    csKalmanRes{filteringIdx}.sKalmanMatrices   = sKalmanMatrices{modelIdx};
    csKalmanRes{filteringIdx}.sInitValues       = sInitValues{modelIdx};
    %csKalmanRes{filteringIdx}.scenarioIdx       = 1;
    csKalmanRes{filteringIdx}.tVec              = sScenario.y_tVec;
    csKalmanRes{filteringIdx}.xPlusMean = xPlusMean; csKalmanRes{filteringIdx}.xPlusCov = xPlusCov; csKalmanRes{filteringIdx}.weightFactor = weightFactor;
    csKalmanRes{filteringIdx}.xMinusMean = xMinusMean; csKalmanRes{filteringIdx}.xMinusCov = xMinusCov; csKalmanRes{filteringIdx}.logWeightFactor = logWeightFactor;
    for timeIdx = 1:size(xPlusCov,3)
        csKalmanRes{filteringIdx}.xPlusCovTrace(timeIdx) = trace(xPlusCov(:,:,timeIdx));
    end
end

%% calculate weight after every time-step:
for modelIdx = 1:nModels
    previousWeights(modelIdx) = sInitValues{modelIdx}.weight;
    previousLogWeights(modelIdx) = sInitValues{modelIdx}.logWeight;
end

nTimeSteps  = numel(sScenario.y(1,:));
nFilters    = numel(csKalmanRes);
for timeIdx = 1:nTimeSteps
    for filteringIdx = 1:nFilters
        weightFactors(filteringIdx) = csKalmanRes{filteringIdx}.weightFactor(timeIdx);
        logWeightFactors(filteringIdx) = csKalmanRes{filteringIdx}.logWeightFactor(timeIdx);
    end
    
    if enableNormalizeWeights
        % not necessary but convenient to normaliaze weight factors:
        weightFactors = weightFactors ./ sum(weightFactors);
        logWeightFactors = logWeightFactors - max(logWeightFactors);
    end
    
    if timeIdx > 1
        for filteringIdx = 1:nFilters
            previousWeights(filteringIdx) = csKalmanRes{filteringIdx}.weight(timeIdx - 1);
            previousLogWeights(filteringIdx) = csKalmanRes{filteringIdx}.logWeight(timeIdx - 1);
        end
    end
    
    newWeights = previousWeights .* weightFactors;
    newLogWeights = previousLogWeights + logWeightFactors;
    
    if enableNormalizeWeights
        newWeights = newWeights ./ sum(newWeights);
        newLogWeights = newLogWeights - max(newLogWeights);
    end
    
    for filteringIdx = 1:nFilters
        csKalmanRes{filteringIdx}.weight(timeIdx) = newWeights(filteringIdx);
        csKalmanRes{filteringIdx}.logWeight(timeIdx) = newLogWeights(filteringIdx);
    end
    
end
