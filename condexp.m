function pnew=condexp(logp)
%CONDEXP  Compute p\propto exp(logp);
pmax=max(logp,[],1); P =size(logp,1);
pnew = condp(exp(logp-repmat(pmax,P,1)));

% after logp-repmat(pmax,P,1) highest value at every column is 0.
% so highest value at every column of exp(logp-repmat(pmax,P,1)) is 1 and
% all values are positive
