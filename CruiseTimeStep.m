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
    A(1,1) = -sModelParams.g*sModelParams.Cr - sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    A(1,2) = sModelParams.alpha_n(gear)*sModelParams.Ki*sModelParams.Tm/sModelParams.m;
    A(2,1) = -1;
    A(2,2) = 0;
    
    B = zeros(2,2);
    B(1,1) = sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    B(1,2) = -sModelParams.g;
    B(2,1) = 1;
    B(2,2) = 0;
    
    stateVec_x = stateVec_x + (A*stateVec_x + B*input_u)*ts + [b_k;0];
    
    % what is the control (throttle level?)
    u_k = sModelParams.Kp * (input_u(1) - stateVec_x(1)) + sModelParams.Ki*stateVec_x(2);
    
else
    u_k = sModelParams.Kp * (input_u(1) - stateVec_x(1)) + sModelParams.Ki*stateVec_x(2);
    u_k = min(u_k , 1);
    u_k = max(u_k , 0);
    
    v_k1 =  stateVec_x(1) + (sModelParams.alpha_n(gear)*sModelParams.Tm/sModelParams.m*u_k...
        - sModelParams.g*input_u(2) - sModelParams.g*sModelParams.Cr*stateVec_x(1))*ts + b_k;
    
    z_k1 = stateVec_x(2) + (input_u(1) - stateVec_x(1))*ts;
    
    stateVec_x = [v_k1 ; z_k1];
end

end

