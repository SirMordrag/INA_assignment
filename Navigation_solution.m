clc;
clearvars;
close all;

%% data
% data format: [x, y, z] or [latitude, longitude, altitude]
% data order: acc, gyro, gps, ...
% load data
load("B(E)3(9)M38IMA_Data_Individual_Semester_Work.mat");
data_acc  = [data(:,3),  data(:,5),  data(:,7)];
data_gyro = [data(:,11), data(:,13), data(:,15)];
data_gpsP = [data(:,20), data(:,21), data(:,22)];
data_gpsV = [data(:,28), data(:,29), data(:,27)];

% calculate periods, lengths
T_imu = 1/200;
T_gps = 1/5;
data_length = size(data);
data_length = data_length(1);

% TEST: data length = * freq * seconds * MINUTES
% data_length = 200 * 60 * 5;

%% declare, initialise
% strapdown
[P, V, DCM] = initial_conditions(data_acc, data_gpsP, data_gpsV, 60);
[R_M, R_N, g_N] = get_pseudo_constants(P(1), P(2), P(3));
% correction
f_bias = zeros(3,1);
w_bias = zeros(3,1);
% KF matrices
nx = 15;
nz = 6;
[F_kf, G_kf] = get_model_matrices(P, V, DCM, [0,0,0], [0,0,0], [0,0,0], R_M, R_N, T_gps); % get G matrix
[x_kf, z_kf, P_kf, Q_c, R_kf, K_kf, H_kf, x_kf_predict] = init_kf_matrices(nx, nz, G_kf);
P_kf_predict = P_kf;
% auxiliary
SAVED_DATA = zeros(data_length, 3 + 3 + 9 + 6 + 15 + 90 + 15 + 3 + 3);
innovation_kf = zeros(6,1); x_kf_saved = zeros(15,1);
P_initial = P_kf;
kf_counter = 0;
pitch = 0; roll = 0; yaw = 0; 
%% main loop
fprintf("Procesing %6i samples, corresponding to %.1f minutes of data\n     Done             ", data_length, data_length/12000); tic;
for index = 1:data_length
    % print progress
    if mod(index, 1000) == 0
        fprintf("\b\b\b\b\b\b\b\b\b\b\b\b%6i, %3.0f%%", index, 100 * index/data_length);
    end
    %% A: MEASUREMENT
    f = data_acc(index,:).' * 9.80665; % g to m/s2
    w = data_gyro(index,:).' * 0.0174532925; % deg/s to rad/s

    %% B: MEASUREMENT CORRECTIONS
    % apply bias
    f = f + f_bias;
    w = w + w_bias;

    %% C: NAVIGATION MECHANISATION
    % update CDM
    dDCM = DCM * [  0   -w(3)  w(2)
                   w(3)   0   -w(1)
                  -w(2)  w(1)   0  ];
    DCM = DCM + dDCM * T_imu;
    % update velocity
    f_n = DCM * f;
    dV = f_n + g_N;
    V = V + dV * T_imu;
    % update position
    dP = [ 1/(R_M + P(3))                    0                     0
               0               1/(cos(P(1)) * (R_N + P(3)))        0
               0                             0                    -1 ] * V;
    P = P + dP * T_imu;

    %% D: REFERENCE MEASUREMENT
    % GPS data are our reference
    p_gps = data_gpsP(index,:).';
    v_gps = data_gpsV(index,:).';

    % If reference measurements are valid, full cycle is performed. Otherwise, we go back to state A
    if ~isnan(p_gps(1)) % GPS data are not NaN, good to go for the whole cycle
        % turn GPS of for a moment               -FLAG-
        if (index > 100000 && index < 112000) && true
            P_kf = P_initial;
            P_kf_predict = P_initial;
            SAVED_DATA(index,:) = [P; V; reshape(DCM,[],1); innovation_kf; reshape(diag(P_kf),[],1); reshape(K_kf,[],1); x_kf_saved; f_bias; w_bias];
            continue;
        end
        % update counter
        kf_counter = kf_counter + 1;
        %% E: DELTA MEASUREMENT VECTOR
        % measurement vector is the difference between position from dead reckoning and GPS
        % WARNING: signs
        z_kf = [p_gps - P;  v_gps - V];

        %% F: CORRECTION UPDATE (KF)
        % Kalman Gain Equation
        K_kf = P_kf_predict * H_kf.' / (H_kf * P_kf_predict * H_kf.' + R_kf);

        % Update Estimate Equation
        innovation_kf = z_kf - H_kf * x_kf_predict;
        x_kf = x_kf_predict + K_kf * innovation_kf;

        % Update Estimate Uncertainty Equation
        P_kf = (eye(nx) - K_kf *  H_kf) * P_kf_predict * (eye(nx) - K_kf * H_kf).' + K_kf * R_kf * K_kf.';
        P_kf = 0.5 * (P_kf + P_kf.'); % force symmetry

        % model matrices
        [F_kf, G_kf] = get_model_matrices(P, V, DCM, f_n, w, [pitch, roll, yaw], R_M, R_N, T_gps);
        
        %% G: CORRECTION APPLICATION
        % apply corrections to Position, Velocity, DCM immediately
        P = P + x_kf(1:3);
        V = V + x_kf(4:6);
        [yaw, roll, pitch] = dcm2angle(DCM, "ZYX");
        DCM = angle2dcm(yaw - x_kf(9), roll - x_kf(8), pitch - x_kf(7), "ZYX");

        % corrections for for bias of IMU measurements are dx_a and dx_g from state vector (I think)
        f_bias = f_bias + x_kf(10:12) .* [1 1 1].' /40; % divide by number of samples that went by in IMU (integration)
        w_bias = w_bias + x_kf(13:15)/40;

        %% H: STATE VECTOR RESET
        % set state vector to zeros
        x_kf_saved = x_kf;
        x_kf = zeros(nx,1);

        %% I: TIME UPDATE (KF)
        
        % discretize Q
        Q_kf = 0.5 * T_gps * (F_kf * G_kf * Q_c * G_kf.' + G_kf * Q_c * G_kf.' * F_kf.');

        % state extrapolation
        x_kf_predict = zeros(nx, 1);

        % uncertainty extrapolation
        P_kf_predict = F_kf * P_kf * F_kf.' + Q_kf;
        P_kf_predict = 0.5 * (P_kf_predict + P_kf_predict.'); % force symmetry
    end % KF loop

    % save data for plot
     SAVED_DATA(index,:) = [P; V; reshape(DCM,[],1); innovation_kf; reshape(diag(P_kf),[],1); reshape(K_kf,[],1); x_kf_saved; f_bias; w_bias];

end % main loop
fprintf("\nFinished calculating in %.1f seconds.\n", toc);

%% post processing
fprintf("Plotting data, this may take couple of minutes.\n"); tic;
fprintf("Note: If it takes too long, please set 'is_quick_plotting' to true in plot_saved_data function (line 4). Some parameters (e.g. attitude) will not be plotted.\n")
plot_saved_data(SAVED_DATA, data);
fprintf("Finished plotting in %.1f seconds.\n", toc);

fprintf("Finished.\n");
