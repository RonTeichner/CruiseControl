function [xEstMean, xEstCov, Gp] = LDSbackward(F, xEstfMean, xEstfCov, Q)
%LDSBACKWARD Full Backward Pass for a Latent Linear Dynamical System (RTS correction method)
% [g,G]=LDSbackward(v,A,B,f,F,CovH,meanH) 
%
% Inputs:

% F : transition matrix
% f : forward pass means
% F : forward pass covariances
% CovH : transition covariance

%
% Outputs:
% xEstMean_at_k : smoothed mean of p(h(t|v(1:t))
% xEstCov_at_k : smoothed covariance of p(h(t)|v(1:t))
% Gp : smoothed two step posterior <h_t h_{t+1}'|v(1:T)>
%import brml.*
N = size(xEstfMean,1); T = size(xEstfMean,2);
xEstMean=zeros(N,T); xEstCov=zeros(N,N,T);
xEstMean(:,T)=xEstfMean(:,T); xEstCov(:,:,T)=xEstfCov(:,:,T);

for t=T-1:-1:1
    [xEstMean(:,t) xEstCov(:,:,t) Gp(:,:,t)]=LDSbackwardUpdate(xEstMean(:,t+1),xEstCov(:,:,t+1),xEstfMean(:,t),xEstfCov(:,:,t),F,Q);
end