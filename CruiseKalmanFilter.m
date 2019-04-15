function [xPlusMean , xPlusCov , weightFactor , xMinusMean , xMinusCov] = CruiseKalmanFilter(sKalmanMatrices,sInitValues,measurements,systemExternalInputs)

xPlusMean_init  = sInitValues.xPlusMean_init;
xPlusCov_init   = sInitValues.xPlusCov_init;
uInput_init     = sInitValues.uInput_init;

nSamples = size(measurements,2);

xMinusMean  = zeros(size(sKalmanMatrices.F,2) , nSamples);
xMinusCov   = zeros(size(sKalmanMatrices.F,2) , size(sKalmanMatrices.F,2) , nSamples);
xPlusMean  = zeros(size(sKalmanMatrices.F,2) , nSamples);
xPlusCov   = zeros(size(sKalmanMatrices.F,2) , size(sKalmanMatrices.F,2) , nSamples);
weightFactor = zeros(nSamples,1); % eta / det(H)

% prepare for weight-update:
inv_H = inv(sKalmanMatrices.C);
S_firstPart = inv_H * sKalmanMatrices.R * transpose(inv_H);

K = zeros(size(xPlusCov));
I = eye(2);
for i = 1:nSamples
    % predictor:
    if i > 1
        xMinusMean(:,i) = sKalmanMatrices.F * xPlusMean(:,i-1) + sKalmanMatrices.G * systemExternalInputs(:,i-1);
        xMinusCov(:,:,i) = sKalmanMatrices.F * xPlusCov(:,:,i-1) * transpose(sKalmanMatrices.F) + sKalmanMatrices.Q;
    else % first iteration from init values
        xMinusMean(:,i) = sKalmanMatrices.F * xPlusMean_init + sKalmanMatrices.G * uInput_init;
        xMinusCov(:,:,i) = sKalmanMatrices.F * xPlusCov_init * transpose(sKalmanMatrices.F) + sKalmanMatrices.Q;
    end
    
    % symmetrize P:
    xMinusCov(:,:,i) = 0.5 * (xMinusCov(:,:,i) + transpose(xMinusCov(:,:,i)));
    
    
    % corrector:
    K(:,:,i) = (xMinusCov(:,:,i) * transpose(sKalmanMatrices.C))...
        /(sKalmanMatrices.C * xMinusCov(:,:,i) * transpose(sKalmanMatrices.C) + sKalmanMatrices.R);
    
    xPlusMean(:,i) = xMinusMean(:,i) + K(:,:,i)*(measurements(:,i) - sKalmanMatrices.C * xMinusMean(:,i));
    xPlusCov(:,:,i) = (I - K(:,:,i)*sKalmanMatrices.C) * xMinusCov(:,:,i) * transpose(I - K(:,:,i)*sKalmanMatrices.C)...
        + K(:,:,i)*sKalmanMatrices.R*transpose(K(:,:,i));
    
    % symmetrize P:
    xPlusCov(:,:,i) = 0.5 * (xPlusCov(:,:,i) + transpose(xPlusCov(:,:,i)));
    
    % weight update:
    S = S_firstPart + xMinusCov(:,:,i);
    diffFromMean = inv_H * measurements(:,i) - xMinusMean(:,i);
    eta = (exp(-0.5 * transpose(diffFromMean) / S * (diffFromMean) )) / (sqrt(det(2*pi*S)));
    weightFactor(i) = eta / det(sKalmanMatrices.C);
end


end

