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

%% declare, initialise
% strapdown
[P, V, DCM] = initial_conditions(data_acc, data_gpsP, data_gpsV, 60);
[R_M, R_N, g_N] = get_pseudo_constants(P(1), P(2), P(3));
r = [0 0 0]; % dunno
kd = 1; % dunno
F63 = 1; % dunno
f_correction = 0;
w_correction = 0;
% KF stuff
dx = zeros(12,1); % state error vector
ddx = zeros(12,1); % state error vector change
% todo reaaaaaly not zeros/ones
P_kf = ones(12);
Q_kf = zeros(12);
K_kf = zeros(12, 6); % WARNING dimensions (nx x nz)
H_kf = ones(6, 12); % ditto (nz x nx)
R_kf = zeros(6);
% auxiliary
SAVED_DATA = zeros(data_length, 9);

%% MAIN LOOP
for index = 1:data_length
    %% A: MEASUREMENT
    f = data_acc(index,:).' * 9.81; % g to m2/s
    w = data_gyro(index,:).' * 0.0174532925; % deg/s to rad/s

    %% B: MEASUREMENT CORRECTIONS
    % WARNING signs (!)
    f = f - f_correction;
    w = w - w_correction;

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
        % WARNING: signs / substraction order, dimensions
        delta_P = p_gps - P;
        delta_V = v_gps - V;
        z_kf = [delta_P.'  delta_V.'  0 0 0   0 0 0   0 0 0 ]; % dunno rho and dxa and dxg

        %% F: CORRECTION UPDATE (KF)
        % Kalman Gain
        K_kf = P_kf * H_kf.' * 1/(H_kf * P_kf * H_kf.' + R_kf);

        % update estimate
        innovation_kf = z_kf - H_kf * dx_next;
        dx = dx_next + K_kf * innovation_kf;

        % update estimate uncertainty
        P_kf = (eye(12) - K_kf *  H_kf) * P_kf * (eye(12) - K_kf * H_kf).' + K_kf * R_kf * K_kf.';

        %% G: CORRECTION APPLICATION
        % correction factors for IMU measurements, applied then in state A
        f_correction = 0; % TODO
        w_correction = 0; % TODO

        % apply corrections to Position, Velocity, DCM immediately
        P = P - dx(1:3);
        V = V - dx(4:6);
        DCM = DCM - angle2dcm(dx(9), dx(8), dx(7), "ZYX");

        %% H: STATE VECTOR RESET
        % WARNING: dimensions
        % WARNING: maybe not necessary, depends on state E implementation (I guess)
        dx = zeros(12, 1);

        %% I: TIME UPDATE (KF)
        % matrices
        [F, G] = get_model_matrices(P, V, DCM, f_n, w, r, kd, F63, R_M, R_N);

        % state extrapolation
        % TODO u or q
        dx_next = F * dx + G * u;

        % uncertainty extrapolation
        P_kf = P_kf + Q_kf;

    end

    % save data for plot
    % data should be in the form of n x 1, where total n must be equal to number of columns of SAVED_DATA, initialised above
    [yaw, roll, pitch] = dcm2angle(DCM, "ZYX");
    eul_ang = rad2deg([pitch; roll; yaw]);
    SAVED_DATA(index,:) = [P; V; eul_ang];
end

%% post processing
plot_saved_data(SAVED_DATA, data_gpsP, data_gpsV);
