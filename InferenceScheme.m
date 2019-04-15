function csKalmanRes = InferenceScheme(sScenario , sKalmanMatrices , sInitValues , sSimParams , csAllModels)

controlIndexes = find(sScenario.gearChange ~= 0);
assert(controlIndexes(1)~=1 , 'not supported control at first time-step');
lastUsedIdx = 0;
for i = 1:numel(controlIndexes)
    currentControl = sScenario.gearChange(controlIndexes(i));
    % run infer-cont until control:
    indexBeforeControl = controlIndexes(i) - 1;
    
    %csInfer{1}.sModelParams = sScenario.sModelParams;
    csInfer{1}.input_u      = sScenario.input_u(:,lastUsedIdx+1 : indexBeforeControl);
    csInfer{1}.y            = sScenario.y(:,lastUsedIdx+1 : indexBeforeControl);
    csInfer{1}.y_tVec       = sScenario.y_tVec(lastUsedIdx+1 : indexBeforeControl);
    csInfer{1}.y_fs         = sScenario.y_fs;
    
    if i > 1
        for k=1:numel(csKalmanResInferCont)
            sInitValues{k}.xPlusMean_init      = csKalmanRes{k}.xPlusMean(:,lastUsedIdx);
            sInitValues{k}.xPlusCov_init       = csKalmanRes{k}.xPlusCov(:,:,lastUsedIdx);
            sInitValues{k}.uInput_init         = csInfer{1}.input_u(:,1);
        end
    end
    
    csKalmanResInferCont = AllFiltersInferCont(sKalmanMatrices, sInitValues , csInfer{1});
    
    % update results:
    for k=1:numel(csKalmanResInferCont)
        csKalmanRes{k}.kalmanModelIdx(lastUsedIdx+1 : indexBeforeControl)   = csKalmanResInferCont{k}.kalmanModelIdx*ones(1,indexBeforeControl-lastUsedIdx);
        csKalmanRes{k}.tVec(lastUsedIdx+1 : indexBeforeControl)             = csKalmanResInferCont{k}.tVec;
        csKalmanRes{k}.xPlusMean(:,lastUsedIdx+1 : indexBeforeControl)      = csKalmanResInferCont{k}.xPlusMean;
        csKalmanRes{k}.xPlusCov(:,:,lastUsedIdx+1 : indexBeforeControl)     = csKalmanResInferCont{k}.xPlusCov;
        csKalmanRes{k}.weightFactor(lastUsedIdx+1 : indexBeforeControl)     = csKalmanResInferCont{k}.weightFactor;
        csKalmanRes{k}.xPlusCovTrace(lastUsedIdx+1 : indexBeforeControl)    = csKalmanResInferCont{k}.xPlusCovTrace;
        csKalmanRes{k}.weight(lastUsedIdx+1 : indexBeforeControl)           = csKalmanResInferCont{k}.weight;
        currentWeights(k) = csKalmanResInferCont{k}.weight(end);
    end
    
    % run infer-reset:
    %csInfer{1}.sModelParams = sScenario.sModelParams;
    csInferRst{1}.input_u      = sScenario.input_u(:,indexBeforeControl+1);
    csInferRst{1}.y            = sScenario.y(:,indexBeforeControl+1);
    csInferRst{1}.y_tVec       = sScenario.y_tVec(indexBeforeControl+1);
    csInferRst{1}.y_fs         = sScenario.y_fs;
    csInferRst{1}.currentControl = currentControl;
    
    if ~sSimParams.returnToInitValueInReset
        for k=1:numel(csKalmanResInferCont)
            sInitValues{k}.xPlusMean_init      = csKalmanRes{k}.xPlusMean(:,indexBeforeControl);
            sInitValues{k}.xPlusCov_init       = csKalmanRes{k}.xPlusCov(:,:,indexBeforeControl);
            sInitValues{k}.uInput_init         = csInferRst{1}.input_u(:,1);            
        end
    end
    
    for k=1:numel(csKalmanResInferCont)
        sInitValues{k}.weight = csKalmanRes{k}.weight(indexBeforeControl);
    end
    
    csKalmanResInferReset = AllFiltersInferReset(sKalmanMatrices, sInitValues , csInferRst{1} , csAllModels{1}.transitionMat);
    
    % update results:
    for k=1:numel(csKalmanResInferReset)
        csKalmanRes{k}.kalmanModelIdx(indexBeforeControl+1)   = csKalmanResInferReset{k}.kalmanModelIdx;
        csKalmanRes{k}.tVec(indexBeforeControl+1)             = csKalmanResInferReset{k}.tVec;
        csKalmanRes{k}.xPlusMean(:,indexBeforeControl+1)      = csKalmanResInferReset{k}.xPlusMean;
        csKalmanRes{k}.xPlusCov(:,:,indexBeforeControl+1)     = csKalmanResInferReset{k}.xPlusCov;
        %csKalmanRes{k}.weightFactor(indexBeforeControl+1)     = csKalmanResInferReset{k}.weightFactor;
        csKalmanRes{k}.xPlusCovTrace(indexBeforeControl+1)    = csKalmanResInferReset{k}.xPlusCovTrace;
        csKalmanRes{k}.weight(indexBeforeControl+1)           = csKalmanResInferReset{k}.weight;        
    end    
    lastUsedIdx = indexBeforeControl+1;
end