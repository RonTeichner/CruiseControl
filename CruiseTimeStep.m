function [sStateVec,u_k] = CruiseTimeStep(sStateVec, vRef, sModelParams,sSimParams, theta, gear, e_k, b_k)
% Inputs:
% theta - road angle [rad]
% vRef - reference speed [m/s]
% gear - 1...5
% sStateVec
%
% Ron Teichner, 05.04.2019

ts = 1/sSimParams.fs;

% measurement noise:
%e_k = sModelParams.std_e * randn; % [m/s]

% control:
u_k = sModelParams.Kp*(vRef - sStateVec.v - e_k) + sModelParams.Ki*sStateVec.z;

% non-linearity of the controller:
u_k = max(min(u_k , 1) , 0);
z_k1 = sStateVec.z + (vRef - sStateVec.v - e_k)*ts;

% torque:
T = sModelParams.Tm*(1 - sModelParams.beta*(sModelParams.alpha_n(gear)*sStateVec.v/sModelParams.omega_m - 1)^2);

% wind noise:
%b_k = sModelParams.std_b * randn; % [m/s]

% forces:
Fg = sModelParams.m * sModelParams.g * sin(theta);
Fr = sModelParams.m * sModelParams.g * sModelParams.Cr * sStateVec.v;
Fa = 0.5 * sModelParams.rho * sModelParams.Cd * sModelParams.A * sStateVec.v^2;
Fd = Fg + Fr + Fa;

v_k1 = sStateVec.v + (sModelParams.alpha_n(gear) * u_k * T - Fd)/sModelParams.m*ts + b_k*ts;

sStateVec.v = v_k1;
sStateVec.z = z_k1;

end

