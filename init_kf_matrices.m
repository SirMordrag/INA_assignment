function [x_kf, z_kf, P_kf, Q_c, R_kf, K_kf, H_kf, x_kf_predict] = init_kf_matrices(nx, nz, G)
    
    % state & prediction & measurement vectors
    x_kf = zeros(nx, 1);   
    x_kf_predict = x_kf;

    %  vector
    z_kf = zeros(nz, 1);    

    % Process Noise Uncertainty
    % noise vector
    % TODO actual values
    % TUNING
    ni_a = [0.003  0.0036 0.0039]; % ACC noise
    ni_g = [0.0924 0.1242 0.0970]; % GYRO noise
    w_a = 0.01 * [1 1 1]; % ACC bias
    w_g = 0.01 * [1 1 1]; % GYRO bias
    q = [ni_a ni_g w_a w_g].';
    Q_c = q .* eye(12);

    % Estimate Uncertainty
    % initially set as Q_discreet    
    P_kf = G * Q_c * G.';

    % Observation Matrix
    H_kf = eye(nz, nx);

    % Kalman Gain
    K_kf = zeros(nx, nz);

    % Measurement Uncertainty
    % TUNING
    r_p = [0.00001 0.00002 0.1]; % position error (IMU & GPS) [°lat °lng m]
    r_v = [0.1 0.1 0.1]; % velocity error (IMU & GPS) [m/s]
    R_kf = [r_p r_v].' .* eye(nz);
    
end