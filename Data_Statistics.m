% Get some useful statisticts for tuning KF

load("B(E)3(9)M38IMA_Data_Individual_Semester_Work.mat");

data_length = 25000
T_imu = 1/200;
data_time = data_length* T_imu;

data_acc  = [data(1:data_length,3), data(1:data_length,5), data(1:data_length,7)];
data_gyro = [data(1:data_length,11), data(1:data_length,13), data(1:data_length,15)];
data_gpsP = [data(1:data_length,20), data(1:data_length,21), data(1:data_length,22)];
data_gpsV = [data(1:data_length,28), data(1:data_length,29), data(1:data_length,27)];
data_gpsP(any(isnan(data_gpsP),2),:) = []; 
data_gpsV(any(isnan(data_gpsV),2),:) = [];

% plot to check
% figure;
% plot(data_gpsP(:,1), data_gpsP(:,2))
% figure;
% yyaxis left
% plot(data_gpsP(:,1))
% yyaxis right
% plot(data_gpsP(:,2))
% figure;
% plot(data_gpsV)

sigma_acc = sqrt(var(data_acc));
sigma_gyro = sqrt(var(data_gyro));
sigma_acc_SI = sqrt(var(data_acc * 9.80665));
sigma_gyro_SI = sqrt(var(data_gyro * 0.0174532925));
sigma_gps_P = sqrt(var(data_gpsP));
sigma_gps_V = sqrt(var(data_gpsV));

mean_acc = mean(data_acc);
mean_gyro = mean(data_gyro);
mean_gps_P = mean(data_gpsP);
mean_gps_V = mean(data_gpsV);

% pitch & roll from ACC
f_mean = mean(data_acc);
pitch = 0; %atan2(f_mean(1), f_mean(3))
roll  = 0; %atan2(f_mean(2), f_mean(3))
yaw = deg2rad(270); % uneducated guess
P = data_gpsP(1,:).';
V = 0 * data_gpsV(1,:).';
[R_M, R_N, g_N] = get_pseudo_constants(P(1), P(2), P(3));

DCM = angle2dcm(yaw, roll, pitch, 'ZYX');

for i = 1:data_length
    f = data_acc(i,:).' * 9.80665; % g to m/s2

    f_n = DCM * f;
    dV = f_n + g_N;
    V = V + dV * T_imu;
    % update position
    dP = [ 1/(R_M + P(3))                    0                     0
               0               1/(cos(P(1)) * (R_N + P(3)))        0
               0                             0                    -1 ] * V;
    P = P + dP * T_imu;
end

for i = 1:data_length
    w = data_gyro(i,:).' * 0.0174532925; % deg/s to rad/s

    % update DCM
    dDCM = DCM * [  0   -w(3)  w(2)
                   w(3)   0   -w(1)
                  -w(2)  w(1)   0  ];
    DCM = DCM + dDCM * T_imu;
end

P_diff = P - mean(data_gpsP()).';
P_diff_per_s = P_diff / data_time;
V_diff = V - mean(data_gpsV()).';
V_diff_per_s = V_diff / data_time;
[yaw1, roll1, pitch1] = dcm2angle(DCM, "ZYX");
R_diff = [pitch1 - pitch; roll1 - roll; yaw1 - yaw];
R_diff_per_s = R_diff / data_time;

P_diff_per_KF_period = sqrt((P_diff_per_s / 5).^2);
V_diff_per_KF_period = sqrt((V_diff_per_s / 5).^2);
R_diff_per_KF_period = sqrt((R_diff_per_s / 5).^2);

mean_acc_SI = (mean_acc * 9.80665) + g_N.';
mean_gyro_SI = (mean_gyro * 0.0174532925);

fprintf("[R] Dead reckoning bias per 40 IMU samples\n")
fprintf("\t Position (e-5) \n")
disp(P_diff_per_KF_period.' .* [1e5, 1e5, 1]);
fprintf("\t Velocity\n")
disp(V_diff_per_KF_period.');
fprintf("\t Angles (roll/pitch/yaw)\n")
disp(R_diff_per_KF_period.');

fprintf("[Q] Sigma ~ noise\n")
fprintf("\t ACC\n")
disp(sigma_acc_SI);
fprintf("\t GYRO\n")
disp(sigma_gyro_SI);
fprintf("[Q] Mean ~ bias - expecting zero acceleration, zero angular motion\n")
fprintf("\t ACC\n")
disp(mean_acc_SI);
fprintf("\t GYRO\n")
disp(mean_gyro_SI);
