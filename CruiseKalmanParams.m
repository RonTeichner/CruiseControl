function sKalmanMatrices = CruiseKalmanParams(sModelParams,gear,y_fs)

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
    
    ts = 1/y_fs; % [sec]
    
    sKalmanMatrices.F = exp(ts*A);
    I = eye(size(A));
    sKalmanMatrices.G = sKalmanMatrices.F * (I - exp(-ts*A))*inv(A)*B;
    
    sKalmanMatrices.C = [1 0 ; 0 1];
    
    sKalmanMatrices.Q = [max(1e-6 , sModelParams.std_b^2) , 0 ; 0 , max(1e-6 , sModelParams.std_e^2)];