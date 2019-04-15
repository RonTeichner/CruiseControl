function csKalmanRes = InferenceScheme(sScenario , sKalmanMatrices , sInitValues , sSimParams , csAllModels)

controlIndexes = find(sScenario.gearChange ~= 0);
if isempty(controlIndexes) % no external control
    csKalmanRes = AllFiltersInferCont(sKalmanMatrices, sInitValues , sScenario);
else
    controlIndexes = [controlIndexes , size(sScenario.y,2)+1];
    assert(controlIndexes(1)~=1 , 'not supported control at first time-step');
    lastUsedIdx = 0;
    for i = 1:numel(controlIndexes)
        
        %% run infer-cont until control:
        indexBeforeControl = controlIndexes(i) - 1;
        
        %csInfer{1}.sModelParams = sScenario.sModelParams;
        clear sScenarioCropped
        sScenarioCropped.input_u      = sScenario.input_u(:,lastUsedIdx+1 : indexBeforeControl);
        sScenarioCropped.y            = sScenario.y(:,lastUsedIdx+1 : indexBeforeControl);
        sScenarioCropped.y_tVec       = sScenario.y_tVec(lastUsedIdx+1 : indexBeforeControl);
        sScenarioCropped.y_fs         = sScenario.y_fs;
        
        if i > 1
            for k=1:numel(csKalmanResInferCont)
                sInitValuesForContInfer{k}.xPlusMean_init      = csKalmanRes{k}.xPlusMean(:,lastUsedIdx);
                sInitValuesForContInfer{k}.xPlusCov_init       = csKalmanRes{k}.xPlusCov(:,:,lastUsedIdx);
                sInitValuesForContInfer{k}.uInput_init         = sScenarioCropped.input_u(:,1);
                sInitValuesForContInfer{k}.weight              = csKalmanRes{k}.weight(lastUsedIdx);
            end
        else
            sInitValuesForContInfer = sInitValues;
        end
        
        
        csKalmanResInferCont = AllFiltersInferCont(sKalmanMatrices, sInitValuesForContInfer , sScenarioCropped);
        
        % update results:
        for k=1:numel(csKalmanResInferCont)
            csKalmanRes{k}.kalmanModelIdx(lastUsedIdx+1 : indexBeforeControl)   = csKalmanResInferCont{k}.kalmanModelIdx*ones(1,indexBeforeControl-lastUsedIdx);
            csKalmanRes{k}.tVec(lastUsedIdx+1 : indexBeforeControl)             = csKalmanResInferCont{k}.tVec;
            csKalmanRes{k}.xPlusMean(:,lastUsedIdx+1 : indexBeforeControl)      = csKalmanResInferCont{k}.xPlusMean;
            csKalmanRes{k}.xPlusCov(:,:,lastUsedIdx+1 : indexBeforeControl)     = csKalmanResInferCont{k}.xPlusCov;
            csKalmanRes{k}.xMinusMean(:,lastUsedIdx+1 : indexBeforeControl)      = csKalmanResInferCont{k}.xMinusMean;
            csKalmanRes{k}.xMinusCov(:,:,lastUsedIdx+1 : indexBeforeControl)     = csKalmanResInferCont{k}.xMinusCov;
            csKalmanRes{k}.weightFactor(lastUsedIdx+1 : indexBeforeControl)     = csKalmanResInferCont{k}.weightFactor;
            csKalmanRes{k}.xPlusCovTrace(lastUsedIdx+1 : indexBeforeControl)    = csKalmanResInferCont{k}.xPlusCovTrace;
            csKalmanRes{k}.weight(lastUsedIdx+1 : indexBeforeControl)           = csKalmanResInferCont{k}.weight;
            currentWeights(k) = csKalmanResInferCont{k}.weight(end);
        end
        
        %% run infer-reset:
        clear sScenarioCropped
        %csInfer{1}.sModelParams = sScenario.sModelParams;
        if indexBeforeControl+1 > size(sScenario.y,2)
            break
        end
        currentControl = sScenario.gearChange(controlIndexes(i));
        
        sScenarioCropped.input_u      = sScenario.input_u(:,indexBeforeControl+1);
        sScenarioCropped.y            = sScenario.y(:,indexBeforeControl+1);
        sScenarioCropped.y_tVec       = sScenario.y_tVec(indexBeforeControl+1);
        sScenarioCropped.y_fs         = sScenario.y_fs;
        sScenarioCropped.currentControl = currentControl;
        
        if ~sSimParams.returnToInitValueInReset
            for k=1:numel(csKalmanResInferCont)
                sInitValuesForInferReset{k}.xPlusMean_init      = csKalmanRes{k}.xPlusMean(:,indexBeforeControl);
                sInitValuesForInferReset{k}.xPlusCov_init       = csKalmanRes{k}.xPlusCov(:,:,indexBeforeControl);
                sInitValuesForInferReset{k}.uInput_init         = sScenarioCropped.input_u(:,1);
            end
        else
            sInitValuesForInferReset = sInitValues;
        end
        
        for k=1:numel(csKalmanResInferCont)
            sInitValuesForInferReset{k}.weight = csKalmanRes{k}.weight(indexBeforeControl);
        end
        
        csKalmanResInferReset = AllFiltersInferReset(sKalmanMatrices, sInitValuesForInferReset , sScenarioCropped , csAllModels{1}.transitionMat);
        
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
end