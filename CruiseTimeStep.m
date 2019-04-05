function [sStateVec,u] = CruiseTimeStep(sStateVec, vRef, sParams, theta, gear)
% Inputs:
% theta - road angle [rad]
% vRef - reference speed [m/s]
% gear - 1...5
% sStateVec
%
% Ron Teichner, 05.04.2019


% measurement noise:
e_k = sParams.std_e * randn; % [m/s]

% control:
z_k1 = vRef - sStateVec.v - e_k;
u = Kp*z_k1 + Ki*sStateVec.z;

% torque:
T = sParams.Tm*(1 - sParams.beta*(sParams.alpha_n(gear)*sStateVec.v/sParams.omega_m - 1)^2);

% wind noise:
b_k = sParams.std_b * randn; % [m/s]

% forces:
Fg = sParams.m * sParams.g * sin(theta);
Fr = sParams.m * sParams.g * sParams.Cr * sStateVec.v;
Fa = 0.5 * sParams.rho * sParams.Cd * sParams.A * sStateVec.v^2;
Fd = Fg + Fr + Fa;

v_k1 = (sParams.alpha_n(gear) * u * T - Fd)/m + b_k;

sStateVec.v = v_k1;
sStateVec.z = z_k1;

end

