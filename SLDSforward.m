function [xEstMean, xEstCov, alpha, w, loglik]=SLDSforward(y,u,F,G,H,Q,R,xInitCov,xInitMean,tranS,priorS,I)
%SLDSFORWARD Switching Latent Linear Dynamical System Gaussian Sum forward pass
%  [f F alpha w loglik]=SLDSforward(v,A,B,CovH,CovV,meanH,meanV,CovP,meanP,tranS,priorS,I)
%

% define:   N - number of state variables
%           S - number of switched states
%           U - number of entries in input control vec
%           T - number of time-steps
%           Y - number of entries in observation vec

% Inputs:
% y             : observations                  - [Y x T]
% u             : input control                 - [U x T]
% F             : transition-autonumous matrix  - [N x N x S]
% G             : transition control            - [N x U x S]
% H             : emission matrix               - [Y x N x S]
% Q             : transition covariance         - [N x N x S]
% R             : emission covariance           - [Y x Y x S]
% xInitCov      : initial prior                 - [N x N]
% xInitMean     : initial mean                  - [N x 1]
% transS        : switch transition distribution- [S x S]
% prior         : switch initial distribution   - [S x 1]
% I             : number of Gaussian components in the Gaussian Sum approximation
%
% Outputs:
% xEstMean      : filterered mean p(h(t|v(1:t))
% xEstCov       : filterered covariance p(h(t)|v(1:t))
% alpha         : filtered switch distribution p(s(t)|v(1:t))
% w             : mixture weights
% loglik        : log likelihod of the sequence log p(v(1:T))

% See also SLDSbackward.m, demoSLDStraffic.m
import brml.*
S=size(F,3); T=size(y,2); N=size(F,1); % N - number of state variables
w=zeros(I,S,T); loglik=0;
xEstMean=zeros(N,I,S,T); xEstCov=zeros(N,N,I,S,T);

% meanF : transition mean
% meanV : emission mean
% meanF = zeros(N,S);
% meanV = zeros(size(y,1),S);

% first time-step (t=1)
for s=1:S
    u_at_k_minus1 = zeros(size(u(:,1)));
    [xEstMean(:,1,s,1), xEstCov(:,:,1,s,1), logphat] = LDSforwardUpdate(...
                        xInitMean, xInitCov, ...
                        y(:,1), u_at_k_minus1, ...
                        F(:,:,s), G(:,:,s), H(:,:,s), Q(:,:,s), R(:,:,s));
        
    logalpha(s,1) = sumlog(priorS(s))+logphat;
end
alpha(:,1)=condexp(logalpha);
w(1,:,1)=1;
% remaining time-steps:
for t=2:T
    if t==2;Itm=1; else Itm=I;end
    for st=1:S
        ind=0;
        for i=1:Itm
            for s=1:S
                ind=ind+1; % ind represents a specific combination of state and gaussian component
                % previous state was s and gaussian component was i, current state is st:                
                [mu(:,ind), Sigma(:,:,ind), logphat] = LDSforwardUpdate(...
                    xEstMean(:,i,s,t-1), xEstCov(:,:,i,s,t-1),...
                    y(:,t), u(:,t-1), ...
                    F(:,:,st), G(:,:,st), H(:,:,st), Q(:,:,st), R(:,:,st));
                
                logp(st,ind) = sumlog([w(i,s,t-1) tranS(st,s) alpha(s,t-1)]) + logphat;
            end
        end
        % collapse:
        [w(:,st,t), xEstMean(:,:,st,t), xEstCov(:,:,:,st,t)] = mix2mix(condexp(logp(st,:)'), mu, Sigma, I);
        logalpha(st) = logsumexp(logp(st,:),ones(1,ind));
        %if mean(abs(f(:)))>100000; keyboard; end
    end
    alpha(:,t) = condexp(logalpha);
    loglik = loglik + logsumexp(logp(:),ones(S*ind,1));
end