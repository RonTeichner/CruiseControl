function csKalmanRes = AllFiltersInferReset(sKalmanMatrices, sInitValues , sScenario , transitionMat)

nModels = numel(sKalmanMatrices);

for modelIdx = 1:nModels
   allWeights(modelIdx) = sInitValues{modelIdx}.weight;
   allLogWeights(modelIdx) = sInitValues{modelIdx}.logWeight;
end

for modelIdx = 1:nModels
    csKalmanResSingleInferReset = SingleFilterInferReset(sKalmanMatrices{modelIdx} , allWeights, allLogWeights , sInitValues{modelIdx} , sScenario , transitionMat);
    csKalmanRes{modelIdx}       = csKalmanResSingleInferReset;
    allNewWeights(modelIdx)     = csKalmanResSingleInferReset.weight;
    allNewLogWeights(modelIdx)  = csKalmanResSingleInferReset.logWeight;
end

% normalize weights:
allNewWeights = allNewWeights ./ sum(allNewWeights);
allNewLogWeights = allNewLogWeights - max(allNewLogWeights);
for modelIdx = 1:nModels
    csKalmanRes{modelIdx}.weight = allNewWeights(modelIdx);
    csKalmanRes{modelIdx}.logWeight = allNewLogWeights(modelIdx);
end
