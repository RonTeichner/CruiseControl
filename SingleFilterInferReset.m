function csKalmanResSingleInferReset = SingleFilterInferReset(sKalmanMatricesRst , allWeightsRst, allLogWeightsRst , sInitValuesRst , sScenario , transitionMat)

% calculate the gaussian using infer cont:
sInitValues{1} = sInitValuesRst; sKalmanMatrices{1} = sKalmanMatricesRst;
enableNormalizeWeights = false;
csKalmanResInferCont                        = AllFiltersInferCont(sKalmanMatrices, sInitValues , sScenario , enableNormalizeWeights);

csKalmanResSingleInferReset.kalmanModelIdx  = csKalmanResInferCont{1}.kalmanModelIdx;
csKalmanResSingleInferReset.tVec            = csKalmanResInferCont{1}.tVec;
csKalmanResSingleInferReset.xPlusMean       = csKalmanResInferCont{1}.xPlusMean;
csKalmanResSingleInferReset.xPlusCov        = csKalmanResInferCont{1}.xPlusCov;
csKalmanResSingleInferReset.xMinusMean       = csKalmanResInferCont{1}.xMinusMean;
csKalmanResSingleInferReset.xMinusCov        = csKalmanResInferCont{1}.xMinusCov;
csKalmanResSingleInferReset.xPlusCovTrace   = csKalmanResInferCont{1}.xPlusCovTrace;

% weight update:
xMinus = csKalmanResSingleInferReset.xMinusMean;
pMinus = csKalmanResSingleInferReset.xMinusCov;


transitionVec = transpose(transitionMat(:,sKalmanMatricesRst.modelIdx));
allWeightsRst = transpose(allWeightsRst);
delta = (transitionVec*allWeightsRst);

logTransitionVec = log(transitionVec);
logMult = logTransitionVec + allLogWeightsRst;
logDelta = log(sum(exp(logMult)));

gaussianMean = sKalmanMatricesRst.C*xMinus;
gauusianCov = sKalmanMatricesRst.C * pMinus * transpose(sKalmanMatricesRst.C) + sKalmanMatricesRst.R;

% sample y from the gauusian:
y = sScenario.y;
yProb = (1/sqrt( (2*pi)^2 * det(gauusianCov) )) * exp(-0.5 * transpose(y - gaussianMean) * inv(gauusianCov) * (y - gaussianMean));
log_yProb = - log(2*pi) - 0.5*log(det(gauusianCov)) + (-0.5 * transpose(y - gaussianMean) * inv(gauusianCov) * (y - gaussianMean));

csKalmanResSingleInferReset.weight = yProb * delta;
csKalmanResSingleInferReset.logWeight = log_yProb + logDelta;