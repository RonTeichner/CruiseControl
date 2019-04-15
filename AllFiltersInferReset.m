function csKalmanRes = AllFiltersInferReset(sKalmanMatrices, sInitValues , sScenario , transitionMat)

nModels = numel(sKalmanMatrices);

for modelIdx = 1:nModels
   allWeights(modelIdx) = sInitValues{modelIdx}.weight;
end

for modelIdx = 1:nModels
    csKalmanResSingleInferReset = SingleFilterInferReset(sKalmanMatrices{modelIdx} , allWeights, sInitValues{modelIdx} , sScenario , transitionMat);
    csKalmanRes{modelIdx}       = csKalmanResSingleInferReset;
    allNewWeights(modelIdx)     = csKalmanResSingleInferReset.weight;
end

% normalize weights:
allNewWeights = allNewWeights ./ sum(allNewWeights);
for modelIdx = 1:nModels
    csKalmanRes{modelIdx}.weight = allNewWeights(modelIdx);
end
