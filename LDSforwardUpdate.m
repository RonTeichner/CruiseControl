function [x_plus_at_k, P_plus_at_k, logpvgv, myLogWeightFactor]=LDSforwardUpdate(x_plus_at_k_minus1,P_plus_at_k_minus1,y_k,u_at_k_minus1,F,G,H,Q,R)
%LDSFORWARDUPDATE Single Forward update for a Latent Linear Dynamical System (Kalman Filter)

%
% Inputs:
% x_plus_at_k_minus1    : filterered mean p(h(t)|v(1:t))
% P_plus_at_k_minus1    : filterered covariance p(h(t)|v(1:t))
% y_k                   : observation
% u_at_k_minus1         : control
% F                     : transition-autonomous matrix
% G                     : transition-control matix
% H                     : emission matrix
% Q                     : transition covariance
% R                     : emission covariance

% Outputs:
% x_plus_at_k           : filterered mean p(h(t+1)|v(1:t+1))
% P_plus_at_k           : filterered covariance p(h(t+1)|v(1:t+1))
% logpgvg               : log p(v(t+1)|v(1:t))
% myLogWeightFactor     : weight update factor (log)

% meanF : transition mean
% meanV : emission mean
meanF = zeros(size(x_plus_at_k_minus1)); meanV = zeros(size(y_k(:,1)));

if isvector(Q); Q=diag(Q); end
if isvector(R); R=diag(R); end
if isvector(F); F=diag(F); end

x_minus_at_k = F*x_plus_at_k_minus1 + meanF + G*u_at_k_minus1;

muv = H*x_minus_at_k + meanV;

P_minus_at_k = F*P_plus_at_k_minus1*F' + Q;

Svv = H*P_minus_at_k*H' + R;

Svh = H*P_minus_at_k;

del = y_k - muv;
invSvvdel = Svv\del; % inv(Svv)*del = inv(Svv)*(y_k-muv) = inv(H*P_minus_at_k*H' + R)*(y_k - H*x_minus_at_k - meanV) 

% Svh'*invSvvdel = P_minus_at_k'*H'*inv(H*P_minus_at_k*H' + R) * (y_k - H*x_minus_at_k - meanV)
% this:  P_minus_at_k'*H'*inv(H*P_minus_at_k*H' + R)  should be the Kalman
% Gain. This: P_minus_at_k'*H' should be P_minus_at_k*H'. Since P_minus_at_k
% is simetric it is true. So the expression for x_plus_at_k is OK.

x_plus_at_k = x_minus_at_k + Svh'*invSvvdel;

%Fnew=Shh-Svh'*(Svv\Svh); Fnew=0.5*(Fnew+Fnew');

% Kalman Gain is calculated as I understand it from Simon:
K = P_minus_at_k*H'/Svv; %  P_minus_at_k*H'*inv(Svv) = P_minus_at_k*H'*inv(H*P_minus_at_k*H' + R)

tmp = eye(size(F)) - K*H;
% P_plus_at_k is calculated as I understand from Simon:
P_plus_at_k = tmp*P_minus_at_k*tmp' + K*R*K'; % Joseph's form 

logpvgv = -0.5*del'*invSvvdel - 0.5*log(det(Svv)) - size(y_k,1)*0.5*log(2*pi);
% is it the same as the weight I calculated?
% Here is my calculation:
inv_H = inv(H);
S_firstPart = inv_H * R * transpose(inv_H);
S = S_firstPart + P_minus_at_k;
diffFromMean = inv_H * y_k - x_minus_at_k;
eta = (exp(-0.5 * transpose(diffFromMean) / S * (diffFromMean) )) / (sqrt(det(2*pi*S)));
logEta = (-0.5 * transpose(diffFromMean) / S * (diffFromMean) ) - log(sqrt(det(2*pi*S)));
myWeightFactor = eta / det(H);
myLogWeightFactor = logEta - log(det(H));