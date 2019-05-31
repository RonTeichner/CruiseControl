function [xEstMean, xEstCov, gamma, u] = SLDSbackward(switchTimes, xEstfMean, xEstfCov, xEstfMinus_mean, xEstfMinus_cov, alpha, w, F, Q, tranS, I, J, varargin)
%SLDSBACKWARD  Backward pass using a Mixture of Gaussians
% [g G gamma u]=SLDSbackward(v,f,F,rho,w,A,CovH,meanH,tranS,I,J,<1/0>)
% if optional argument=0 then do Generalised Pseudo Bayes, otherwise Expectation Correction
%

% define:   N - number of state variables
%           S - number of switched states
%           T - number of time-steps
%           I - number of Gaussian components in the Forward Gaussian Sum approximation
% Inputs:
% switchTimes
% xEstfMean         : filterered mean p(h(t|v(1:t))                     - [N x I x S x T]
% xEstfCov          : filterered covariance p(h(t)|v(1:t))              - [N x N x I x S x T]
% xEstfMinus_mean   : filtered mean prior to measurement recieved       - [N x I x S x S x T] dim(3) is previous state, dim(4) is current
% xEstfMinus_cov    : filtered cov prior to measurement recieved        - [N x N x I x S x S x T] dim(4) is previous state, dim(5) is current
% alpha             : filtered switch distribution p(s|v(1:t))          - [S x T]
% w                 : mixture weights                                   - [I x S x T]
% F                 : transition matrix                                 - [N x N]
% Q                 : transition covariance                             - [N x N]

% transS        : switch transition distribution
% I             : number of Gaussian components in the Forward Gaussian Sum approximation
% J             : number of Gaussian components in the Backward Gaussian Sum approximation
%
% Outputs:
% g             : smoothed mean p(h(t)|v(1:T))
% G             : filterered covariance p(h(t)|v(1:T))
% gamma         : smoothed switch distribution p(s(t)|v(1:T))
% u             : mixture weights
% See also SLDSforward.m, demoSLDStraffic.m
%import brml.*
if isempty(varargin)
    doEC=1;
else
    doEC=varargin{1};
end


S = size(F,3); T = size(xEstfMean,4); N = size(F,1);
u = zeros(J,S,T);
xEstMean = zeros(N,J,S,T); xEstCov = zeros(N,N,J,S,T);

% meanH : transition mean
meanH = zeros(N , S);

gamma(:,T)=alpha(:,T);
if J<I
    for st=1:S
        [u(:,st,T),xEstMean(:,:,st,T),xEstCov(:,:,:,st,T)]=mix2mix(w(:,st,T),xEstfMean(:,:,st,T),xEstfCov(:,:,:,st,T),J);
    end
else
    xEstMean(:,:,:,T)=xEstfMean(:,:,:,T); xEstCov(:,:,:,T)=xEstfCov(:,:,:,T); u(:,:,T)=w(:,:,T);
end

for t=T-1:-1:1
    if t==1;It=1; else It=I;end
    Jtp=J;
    if any(t==switchTimes); switchFlag = true; else switchFlag = false; end
    clear logtmp2
    for st=1:S
        for it=1:It
            for stp=1:S
                if switchFlag
                    switchProb = tranS(stp,st);
                else
                    switchProb = (stp == st);
                end
                
                pststpgV1t(it,st,stp) = switchProb * w(it,st,t) * alpha(st,t);
                Stp = F(:,:,stp)*xEstfCov(:,:,it,st,t)*F(:,:,stp)'+Q(:,:,stp);
                logdet2piStp = logdet(2*pi*Stp);
                tmpvec=F(:,:,stp)*xEstfMean(:,it,st,t)+meanH(:,stp);
                for jtp=1:Jtp
                    % LDSbackwardUpdate(xEstMean_at_k_plus1, xEstCov_at_k_plus1, xEstfMean_at_k, xEstfCov_at_k, XfMinus_at_k_plus1, PfMinus_at_k_plus1, F, Q)
                    [mu(:,it,st,jtp,stp),Sigma(:,:,it,st,jtp,stp)] = LDSbackwardUpdate(xEstMean(:,jtp,stp,t+1), xEstCov(:,:,jtp,stp,t+1),...
                        xEstfMean(:,it,st,t), xEstfCov(:,:,it,st,t), xEstfMinus_mean(:,it,st,stp,t+1), xEstfMinus_cov(:,:,it,st,stp,t+1),...
                        F(:,:,stp), Q(:,:,stp));
                    if doEC
                        % compute contribution to mixture weight:
                        % ztp= <htp|stp,jtp,v1:T>-<htp|st,stp,it,v_{1:t}>; Stp is the covariance
                        ztp = xEstMean(:,jtp,stp,t+1) - tmpvec;
                        tmp1=-0.5*ztp(:)'*(Stp\ztp(:))-0.5*logdet2piStp;
                        logtmp2(it,st,jtp,stp)=log(pststpgV1t(it,st,stp)+eps) +tmp1; % Expectation Correction
                    else
                        logtmp2(it,st,jtp,stp)=log(pststpgV1t(it,st,stp)+eps); % Generalised Pseudo Bayes
                    end
                end
            end
        end
    end
    
    pitstgjtpstpV1T=reshape(condexp(reshape(logtmp2,It*S,Jtp*S)),It,S,Jtp,S); % p(st,it|stp,jtp,v1:T)=p(st,it,stp,jtp|v1:T)/p(stp,jtp,v1:T)
    
    for st=1:S
        ind=0;
        gamma(st,t)=0;
        for it=1:It
            for stp=1:S
                for jtp=1:Jtp
                    % p(st,stp,it,jtp|v1:T)=p(stp|V1:T)*p(jtp|stp,v1:T)p(st,it|stp,jtp,V1:T)
                    pmixjoint(it,st,jtp,stp)=gamma(stp,t+1)*u(jtp,stp,t+1)*pitstgjtpstpV1T(it,st,jtp,stp);
                    gamma(st,t)=gamma(st,t) + pmixjoint(it,st,jtp,stp);
                    ind=ind+1;
                    muComp(:,ind)=mu(:,it,st,jtp,stp);
                    SigmaComp(:,:,ind)=Sigma(:,:,it,st,jtp,stp);
                    pComp(ind,1)= pmixjoint(it,st,jtp,stp);
                end
            end
        end
        pComp=condp(pComp);
        [u(:,st,t),xEstMean(:,:,st,t),xEstCov(:,:,:,st,t)]=mix2mix(pComp,muComp,SigmaComp,J);  % Project to a mixture
    end
    
end