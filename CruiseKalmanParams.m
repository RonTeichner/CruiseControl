function sKalmanMatrices = CruiseKalmanParams(sModelParams,y_fs,x_fs)
    gear = sModelParams.gear;
    A = zeros(2,2);
    A(1,1) = -sModelParams.g*sModelParams.Cr - sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    A(1,2) = sModelParams.alpha_n(gear)*sModelParams.Ki*sModelParams.Tm/sModelParams.m;
    A(2,1) = -1;
    A(2,2) = 0;
    
    %disp(['eigen-values of A are: ',mat2str(eig(A))]);
    if max(real(eig(A))) < 0
        %disp('therefore the contineous-time system is stable');
    else
        disp('therefore the contineous-time system is not stable !');
    end
    
    B = zeros(2,2);
    B(1,1) = sModelParams.alpha_n(gear)*sModelParams.Kp*sModelParams.Tm/sModelParams.m;
    B(1,2) = -sModelParams.g;
    B(2,1) = 1;
    B(2,2) = 0;
    
    ts = 1/y_fs; % [sec]
    
    fsFactor = x_fs/y_fs;
    stdFactor = 1.1 * sqrt(fsFactor);
    
    sKalmanMatrices.F = expm(ts*A);
    I = eye(size(A));
    sKalmanMatrices.G = sKalmanMatrices.F * (I - expm(-ts*A))*inv(A)*B;
    
    sKalmanMatrices.C = [1 0 ; 0 1];
    
    sKalmanMatrices.Q = [max(1e-6 , ((stdFactor)*sModelParams.std_b)^2) , 0 ; 0 , max(1e-6 , ((stdFactor)*sModelParams.std_e)^2)];
    
    sKalmanMatrices.R = [max(1e-6 , ((stdFactor)*sModelParams.speedMeasure_std)^2) , 0 ; 0 , max(1e-6 , ((stdFactor)*sModelParams.controllerStateMeasure_std)^2)];
    
    %disp(['eigen-values of F are: ',mat2str(eig( sKalmanMatrices.F))]);
    if max(abs(eig( sKalmanMatrices.F))) < 1
        %disp('therefore the discrete-time system is stable');
    else
        disp('therefore the discrete-time system is not stable !');
    end