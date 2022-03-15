clc;
clear all;
close all;

%% load data
% data format: [x, y, z] or [latitude, longitude, altitude]
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

SAVED_DATA = zeros(data_length, 9);

%% MAIN LOOP
for index = 1:data_length
    %% data extraction
    f = data_acc(index,:).' * 9.81; % g to m2/s
    w = data_gyro(index,:).' * 0.0174532925; % deg/s to rad/s
    p_gps = data_gpsP(index,:);
    v_gps = data_gpsV(index,:);

    % decide if we're doing dead reckoning or KF
    if isnan(p_gps(1)) % GPS data are NAN, let's go to IMU
    %% DEAD RECKONING

        %% input data compensation
        % TODO
        f = f;
        w = w;

        %% navigation equations
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
    else
    %% KALMAN WITH GPS
        %% TODO
        P = data_gpsP(index,:).';
        V = data_gpsV(index,:).';

    end

    % save data for plot
    % data should be in the form of n x 1, where total n must be equal to number of columns of SAVED_DATA, initialised above
    [yaw, roll, pitch] = dcm2angle(DCM, "ZYX");
    eul_ang = rad2deg([pitch; roll; yaw]);
    SAVED_DATA(index,:) = [P; V; eul_ang];
end

%% post processing
plot_saved_data(SAVED_DATA, data_gpsP, data_gpsV);
