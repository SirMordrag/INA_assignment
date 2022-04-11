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
    ni_a = 1 * [1 1 1]; % ACC noise
    ni_g = 1 * [1 1 1]; % GYRO noise
    w_a = 10 * [1 1 1]; % ACC bias
    w_g = 10 * [1 1 1]; % GYRO bias
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
    r_p = [0.0001 0.0001 0.1]; % position error (IMU & GPS) [m]
    r_v = [0.001 0.001 0.001]; % velocity error (IMU & GPS) [m/s]
    R_kf = [r_p r_v].' .* eye(nz);
    
end