function csKalmanRes = InferenceScheme(sScenario , sKalmanMatrices , sInitValues , sSimParams , csAllModels)

enableFigureForDebug = false;

controlIndexes = find(sScenario.gearChange ~= 0);
if isempty(controlIndexes) % no external control
    enableNormalizeWeights = true;
    csKalmanRes = AllFiltersInferCont(sKalmanMatrices, sInitValues , sScenario , enableNormalizeWeights);
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
                sInitValuesForContInfer{k}.uInput_init         = sScenario.input_u(:,lastUsedIdx);
                sInitValuesForContInfer{k}.weight              = csKalmanRes{k}.weight(lastUsedIdx);
                sInitValuesForContInfer{k}.logWeight           = csKalmanRes{k}.logWeight(lastUsedIdx);
            end
        else
            sInitValuesForContInfer = sInitValues;
        end
        
        enableNormalizeWeights = true;
        csKalmanResInferCont = AllFiltersInferCont(sKalmanMatrices, sInitValuesForContInfer , sScenarioCropped , enableNormalizeWeights);
        
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
            csKalmanRes{k}.logWeight(lastUsedIdx+1 : indexBeforeControl)        = csKalmanResInferCont{k}.logWeight;
            currentWeights(k) = csKalmanResInferCont{k}.weight(end);
        end
        
        %% run infer-reset:
        clear sScenarioCropped
        %csInfer{1}.sModelParams = sScenario.sModelParams;
        if indexBeforeControl+1 > size(sScenario.y,2)
            break
        end
        currentControl = sScenario.gearChange(controlIndexes(i));
        
        %sScenarioCropped.input_u      = sScenario.input_u(:,indexBeforeControl+1);
        sScenarioCropped.y            = sScenario.y(:,indexBeforeControl+1);
        sScenarioCropped.y_tVec       = sScenario.y_tVec(indexBeforeControl+1);
        sScenarioCropped.y_fs         = sScenario.y_fs;
        sScenarioCropped.currentControl = currentControl;
        
        if ~sSimParams.returnToInitValueInReset
            for k=1:numel(csKalmanResInferCont) 
                % there is no logic here! we know there was a gear change, 
                % so why think the most probable previous state-vector is
                % from the same kalman?
                % Need to develop more gaussians and collapse them. 
                sInitValuesForInferReset{k}.xPlusMean_init      = csKalmanRes{k}.xPlusMean(:,indexBeforeControl);
                sInitValuesForInferReset{k}.xPlusCov_init       = csKalmanRes{k}.xPlusCov(:,:,indexBeforeControl);
            end
        else
            sInitValuesForInferReset = sInitValues;
        end
        
        
        for k=1:numel(csKalmanResInferCont)
            sInitValuesForInferReset{k}.uInput_init         = sScenario.input_u(:,indexBeforeControl);
            sInitValuesForInferReset{k}.weight              = csKalmanRes{k}.weight(indexBeforeControl);
            sInitValuesForInferReset{k}.logWeight           = csKalmanRes{k}.logWeight(indexBeforeControl);
        end
        
        csKalmanResInferReset = AllFiltersInferReset(sKalmanMatrices, sInitValuesForInferReset , sScenarioCropped , csAllModels{1}.transitionMat);
        
        % update results:
        for k=1:numel(csKalmanResInferReset)
            csKalmanRes{k}.kalmanModelIdx(indexBeforeControl+1)   = csKalmanResInferReset{k}.kalmanModelIdx;
            csKalmanRes{k}.tVec(indexBeforeControl+1)             = csKalmanResInferReset{k}.tVec;
            csKalmanRes{k}.xPlusMean(:,indexBeforeControl+1)      = csKalmanResInferReset{k}.xPlusMean;
            csKalmanRes{k}.xPlusCov(:,:,indexBeforeControl+1)     = csKalmanResInferReset{k}.xPlusCov;
            csKalmanRes{k}.xMinusMean(:,indexBeforeControl+1)      = csKalmanResInferReset{k}.xMinusMean;
            csKalmanRes{k}.xMinusCov(:,:,indexBeforeControl+1)     = csKalmanResInferReset{k}.xMinusCov;
            %csKalmanRes{k}.weightFactor(indexBeforeControl+1)     = csKalmanResInferReset{k}.weightFactor;
            csKalmanRes{k}.xPlusCovTrace(indexBeforeControl+1)    = csKalmanResInferReset{k}.xPlusCovTrace;
            csKalmanRes{k}.weight(indexBeforeControl+1)           = csKalmanResInferReset{k}.weight;
            csKalmanRes{k}.logWeight(indexBeforeControl+1)           = csKalmanResInferReset{k}.logWeight;
        end
        
        
        %% figure during debug:
        if enableFigureForDebug
            figure;
            startTime = sScenario.y_tVec(lastUsedIdx+1); stopTime = sScenario.y_tVec(indexBeforeControl+1);
            [~,startTimeIdx] = min(abs(startTime - sScenario.sGroundTruth.tVec)); [~,stopTimeIdx] = min(abs(stopTime - sScenario.sGroundTruth.tVec));
            kph2m_s = 1000/60/60;
            for gearIdx = 1:5
                gearsIdx{gearIdx} = find(sScenario.sGroundTruth.gears(startTimeIdx : stopTimeIdx) == gearIdx);
            end
            
            ax(1) = subplot(2,1,1); hold all;
            for gearIdx = 1:5
                if any(gearsIdx{gearIdx})
                    plot(sScenario.sGroundTruth.tVec(startTimeIdx-1+gearsIdx{gearIdx}),sScenario.sGroundTruth.stateVec(1,startTimeIdx-1+gearsIdx{gearIdx})./kph2m_s,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
                else
                    plot(0,0,'.','DisplayName',['gear: ',int2str(gearIdx)],'Parent',ax(1)); xlabel('sec'); grid on; ylabel('kph'); title('GroundTruth - speed');
                end
            end
            legend
            xlim([csKalmanRes{1}.tVec(lastUsedIdx+1),csKalmanRes{1}.tVec(indexBeforeControl+1)])
            
            ax(2) = subplot(2,1,2); hold all;
            for filteringIdx = 1:numel(csKalmanRes)
                assert(numel(unique(csKalmanRes{filteringIdx}.kalmanModelIdx))==1)
                log10Weight = (csKalmanRes{filteringIdx}.logWeight) ./ log(10);
                plot(csKalmanRes{filteringIdx}.tVec(lastUsedIdx+1:indexBeforeControl+1) , 20*log10Weight(lastUsedIdx+1:indexBeforeControl+1),'.-','DisplayName',['kModel: ',int2str(csKalmanRes{filteringIdx}.kalmanModelIdx(1))]);
            end
            xlabel('sec'); ylabel('db'); title(['weights']);% true sc model: ',int2str(csSim{1}.modelIdx)]);
            ylim([-100 0]); xlim([csKalmanRes{1}.tVec(lastUsedIdx+1),csKalmanRes{1}.tVec(indexBeforeControl+1)])
            legend
            
            linkaxes(ax,'x');
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        lastUsedIdx = indexBeforeControl+1;
    end
end