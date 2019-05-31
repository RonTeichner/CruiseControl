function [xEstMean_at_k, xEstCov_at_k, Gpnew] = ...
    LDSbackwardUpdate(xEstMean_at_k_plus1, xEstCov_at_k_plus1, xEstfMean_at_k, xEstfCov_at_k, XfMinus_at_k_plus1, PfMinus_at_k_plus1, F, Q)
%LDSBACKWARDUPDATE Single Backward update for a Latent Linear Dynamical System (RTS smoothing update)
% [gnew Gnew Gpnew]=LDSbackwardUpdate(g,G,f,F,A,CovH,meanH)
%
% Inputs:
% xEstMean_at_k_plus1   : smoothed mean p(h(t+1)|v(1:T))
% xEstCov_at_k_plus1    : smoothed covariance p(h(t+1)|v(1:T))
% xEstfMean_at_k        : filterered mean p(h(t)|v(1:t))
% xEstfCov_at_k         : filterered covariance p(h(t)|v(1:t))
% XfMinus_at_k_plus1    : filtered estimation mean for time k+1 before receiving
%                         the measurement y_{k+1}
% PfMinus_at_k_plus1    : filtered estimation cov for time k+1 before receiving
%                         the measurement y_{k+1}
% F                     : transition matrix
% Q                     : transition covariance

%
% Outputs:
% xEstMean_at_k         : smoothed mean p(h(t)|v(1:T))
% xEstCov_at_k          : smoothed covariance p(h(t)|v(1:T))
% Gpnew                 : smoothed cross moment  <h_t h_{t+1}|v(1:T)>

meanH = zeros(size(xEstMean_at_k_plus1));

if isvector(Q); Q=diag(Q); end
if isvector(F); F=diag(F); end

%XfMinus_at_k_plus1     = F*xEstfMean_at_k + G*u_at_k + meanH; % x_minus_f_mean_at_k_plus1

%PfMinus_at_k_plus1      = F*xEstfCov_at_k*F' + Q;
Shtpt   = F*xEstfCov_at_k;

leftA = (Shtpt')/PfMinus_at_k_plus1;                % (xEstfCov_at_k'F')*inv(PfMinus_at_k_plus1);
leftS = xEstfCov_at_k - leftA*Shtpt;
leftm = xEstfMean_at_k - leftA*XfMinus_at_k_plus1;  % xEstfMean_at_k - (xEstfCov_at_k'F')*inv(PfMinus_at_k_plus1)*XfMinus_at_k_plus1 

% KalmanGain    = xEstfCov_at_k' * F' * inv(PfMinus_at_k_plus1);
% xEstMean_at_k = xEstfMean_at_k + KalmanGain*(xEstMean_at_k_plus1 - XfMinus_at_k_plus1)

% I verified that xEstMean_at_k is calculated as in Dan Simon up to
%   xEstfCov_at_k at kalmanGain being without transpose at simon but it
%   should be simetric anyhow.
xEstMean_at_k = leftA*xEstMean_at_k_plus1 + leftm;  

xEstCov_at_k = leftA*xEstCov_at_k_plus1*leftA' + leftS; 
xEstCov_at_k = 0.5*(xEstCov_at_k + xEstCov_at_k'); % could also use Joseph's form if desired

Gpnew = leftA*xEstCov_at_k_plus1 + xEstMean_at_k*xEstMean_at_k_plus1'; % smoothed <h_t h_{t+1}>