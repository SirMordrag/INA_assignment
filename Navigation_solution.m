clc;
clear all;
close all;

%% load data
% data format: [x, y, z] or [latitude, longitude, altitude]
% data order: acc, gyro, gps, ...
load("B(E)3(9)M38IMA_Data_Individual_Semester_Work.mat");
data_acc = [data(:,3), data(:,5), data(:,7)];
data_gyro = [data(:,11), data(:,13), data(:,15)];
data_gpsP = [data(:,20), data(:,21), data(:,22)];
data_gpsV = [data(:,28), data(:,29), data(:,27)];

T_imu = 1/200;
T_gps = 1/5;
data_length = size(data);
data_length = data_length(1);

% TEST: data length = * freq * seconds * MINUTES
data_length = 200 * 60 * 1.5

%% initialise
[P, V, DCM] = initial_conditions(data_acc, data_gpsP, data_gpsV, 60);
[R_M, R_N, g_N] = get_pseudo_constants(P(1), P(2), P(3));
r = [0 0 0]; % dunno
kd = 1; % dunno
F63 = 1; % dunno

SAVED_DATA = zeros(data_length, 9);

%% MAIN LOOP
for index = 1:data_length
    %% A: MEASUREMENT
    f = data_acc(index,:).' * 9.81; % g to m2/s
    w = data_gyro(index,:).' * 0.0174532925; % deg/s to rad/s

    %% B: MEASUREMENT CORRECTIONS
    % TODO
    f = f;
    w = w;

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
    if ~isnan(p_gps(1)) % GPS data are not NaN, good to go for whole cycle
        %% E: DELTA MEASUREMENT VECTOR
        % ideally, those two should be zero
        delta_P = p_gps - P;
        delta_V = v_gps - V;

        %% F: CORRECTION UPDATE (KF)
        % Kalman Gain

        % innovation

        % update estimate

        % update estimate uncertainty

        %% G: CORRECTION APPLICATION


        %% H: STATE VECTOR RESET


        %% I: TIME UPDATE (KF)
        % matrices
        [F, G] = get_model_matrices(P, V, DCM, f_n, w, r, kd, F63, R_M, R_N);

        % state extrapolation

        % uncertainty extrapolation

    end

    % save data for plot
    % data should be in the form of n x 1, where total n must be equal to number of columns of SAVED_DATA, initialised above
    [yaw, roll, pitch] = dcm2angle(DCM, "ZYX");
    eul_ang = rad2deg([pitch; roll; yaw]);
    SAVED_DATA(index,:) = [P; V; eul_ang];
end

%% post processing
plot_saved_data(SAVED_DATA, data_gpsP, data_gpsV);
