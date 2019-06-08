function [stateVec_x,u_k] = CruiseTimeStep(stateVec_x, input_u, sModelParams, sSimParams, gear, b_k, enableLinear)
% Inputs:
% stateVec_x - [v;z]
% input_u - [v_r; sin(theta)]
% sModelParams
% sSimParams
% gear - 1...5
% b_k - process noise due to wind
% enableLinear - control not bounded to [0,1]
% Ron Teichner, 05.04.2019

ts = 1/sSimParams.fs;

if enableLinear
    A = zeros(2,2);
    A(1,1) = -sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    A(1,2) = sModelParams.alpha_n(gear)*sModelParams.Ki*sModelParams.Tm/sModelParams.m;
    A(2,1) = -1;
    A(2,2) = 0;
    
    %     B = zeros(2,2);
    %     B(1,1) = sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    %     B(1,2) = -sModelParams.g;
    %     B(2,1) = 1;
    %     B(2,2) = 0;
    
    B = zeros(2,3);
    B(1,1) = sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    B(1,2) = -sModelParams.g;
    B(1,3) = -sModelParams.g*sModelParams.Cr;
    B(2,1) = 1;
    B(2,2) = 0;
    B(2,3) = 0;
    
    
    input_u = [input_u ; sign(stateVec_x(1))];
    
    stateVec_x = stateVec_x + (A*stateVec_x + B*input_u)*ts + [b_k;0];
    
    % what is the control (throttle level?)
    u_k = sModelParams.Kp * (input_u(1) - stateVec_x(1)) + sModelParams.Ki*stateVec_x(2);
    
else
    u_k = sModelParams.Kp * (input_u(1) - stateVec_x(1)) + sModelParams.Ki*stateVec_x(2);
    u_k = min(u_k , 1);
    u_k = max(u_k , 0);
    
    omega = sModelParams.alpha_n(gear) * stateVec_x(1);
    T = sModelParams.Tm * (1 - sModelParams.beta*(omega/sModelParams.omega_m - 1)^2);
    
    v_k1 =  stateVec_x(1) + ...
        (sModelParams.alpha_n(gear) * u_k / sModelParams.m * T...
        - sModelParams.g * sModelParams.Cr * sign(stateVec_x(1))...
        - 0.5*sModelParams.rho * sModelParams.Cd * sModelParams.A/sModelParams.m * (stateVec_x(1))^2 ...
        - sModelParams.g*input_u(2))*ts...
        + b_k;
    
    z_k1 = stateVec_x(2) + (input_u(1) - stateVec_x(1))*ts;
    
%     % check other derivation:
%     an = sModelParams.alpha_n(gear); om = sModelParams.omega_m; beta = sModelParams.beta; Kp = sModelParams.Kp; Ki = sModelParams.Ki; rho = sModelParams.rho; Cd = sModelParams.Cd; Cr = sModelParams.Cr; A = sModelParams.A;
%     l = sModelParams.Tm * an /sModelParams.m;
%     
%     v_k1_check = stateVec_x(1) + ...
%         (stateVec_x(1)^3 * (an^2/om^2*l*Kp*beta)...
%         -stateVec_x(1)^2 * (l*Kp*(an^2/om^2*input_u(1)*beta + 2*an/om*beta + 0.5*rho*Cd*A/Kp/(l*sModelParams.m)))...
%         -stateVec_x(1)^2*stateVec_x(2) * (an^2/om^2*l*Ki*beta)...
%         +stateVec_x(1) * (l*Kp*(2*an/om*input_u(1)*beta - 1 - beta))...
%         +stateVec_x(1)*stateVec_x(2) * (2*an/om*l*Ki*beta)...
%         +stateVec_x(2) * (l*Ki*(1+beta))...
%         +(l*Kp*input_u(1)*(1-beta) - sModelParams.g * Cr * sign(stateVec_x(1)) - sModelParams.g*input_u(2))) *ts...
%         + b_k;
%     
%     
%     if abs(v_k1_check-v_k1) > 1e-2
%         keyboard
%     end
%     assert(abs(v_k1_check-v_k1) < 1e-1,' my derivation is bad');
    
    stateVec_x = [v_k1 ; z_k1];
    
    
end

end

