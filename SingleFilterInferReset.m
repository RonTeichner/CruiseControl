function csKalmanResSingleInferReset = SingleFilterInferReset(sKalmanMatricesRst , allWeightsRst, sInitValuesRst , sScenario , transitionMat)

% calculate the gaussian using infer cont:
sInitValues{1} = sInitValuesRst; sKalmanMatrices{1} = sKalmanMatricesRst;
csKalmanResInferCont                        = AllFiltersInferCont(sKalmanMatrices, sInitValues , sScenario);

csKalmanResSingleInferReset.kalmanModelIdx  = csKalmanResInferCont{1}.kalmanModelIdx;
csKalmanResSingleInferReset.tVec            = csKalmanResInferCont{1}.tVec;
csKalmanResSingleInferReset.xPlusMean       = csKalmanResInferCont{1}.xPlusMean;
csKalmanResSingleInferReset.xPlusCov        = csKalmanResInferCont{1}.xPlusCov;
csKalmanResSingleInferReset.xPlusCovTrace   = csKalmanResInferCont{1}.xPlusCovTrace;

% weight update:
xMinus = sKalmanMatricesRst.F * sInitValuesRst.xPlusMean_init;
pMinus = sKalmanMatricesRst.F * sInitValuesRst.xPlusCov_init * transpose(sKalmanMatricesRst.F) + sKalmanMatricesRst.Q;


if sScenario.currentControl == -1 % change down
    transitionMat = squeeze(transitionMat(:,:,1));
elseif sScenario.currentControl == 1 % change up
    transitionMat = squeeze(transitionMat(:,:,3));
end

transitionVec = transitionMat(:,sKalmanMatricesRst.modelIdx);

delta = (allWeightsRst*transitionVec) * sInitValuesRst.weight ;

gaussianMean = sKalmanMatricesRst.C*xMinus;
gauusianCov = sKalmanMatricesRst.C * pMinus * transpose(sKalmanMatricesRst.C) + sKalmanMatricesRst.R;

% sample y from the gauusian:
y = sScenario.y;
yProb = (2*pi)^(-1) * (det(gauusianCov))^(-0.5) * exp(-0.5 * transpose(y - gaussianMean) * inv(gauusianCov) * (y - gaussianMean));

csKalmanResSingleInferReset.weight = yProb * delta;