clc;
clear all;
close all;

load("B(E)3(9)M38IMA_Data_Individual_Semester_Work.mat");
data_acc = [data(:,3), data(:,5), data(:,7)];
data_gyro = [data(:,11), data(:,13), data(:,15)];
data_gpsP = [data(:,20), data(:,21), data(:,22)];
data_gpsV = [data(:,28), data(:,29), data(:,27)];

% data convention:
% [x, y, z]
% [latitude, longitude, altitude]
T_sample = 1/200;
data_length = size(data);
data_length = data_length(1);

data_length = 18000


%% initial conditions
% position and velocity from first GPS reading
for i = 1:data_length
    if ~isnan(data_gpsP(i,1))
        P = [data_gpsP(i,1) data_gpsP(i,2) data_gpsP(i,3)].';
        V = [data_gpsV(i,1) data_gpsV(i,2) data_gpsV(i,3)].';
        break;
    end
end

DCM = [1 0 0
       0 1 0
       0 0 1];

%% pseudo constants
g_n = getg_n( P(1), P(3) );
% g_n = [0 0 1].';
R_M = getR_M( P(1) );
R_N = getR_N( P(1) );

SAVEDp = zeros(3, data_length);
SAVEDv = zeros(3, data_length);

%% MAIN LOOP
for index = 1:data_length

    %% input data compensation
    f = compensate_acc( data_acc(index,:) ).' * 9.81; % g to m2/s
    w = compensate_gyro( data_gyro(index,:) ).' * 0.0174532925; % deg/s to rad/s
    
    %% navigation equations
    % update CDM
    dDCM = DCM * [  0   -w(3)  w(2)
                   w(3)   0   -w(1)
                  -w(2)  w(1)   0  ];
    DCM = DCM + dDCM * T_sample;
    % update velocity
    f_n = DCM * f;
    dV = f_n + g_n;
    V = V + dV * T_sample;
    % update position
    dP = [ 1/(R_M + P(3))                    0                     0
               0               1/(cos(P(1)) * (R_N + P(3)))        0
               0                             0                    -1 ] * V;
    P = P + dP * T_sample;

    SAVEDp(1, index) = P(1);
    SAVEDp(2, index) = P(2);
    SAVEDp(3, index) = P(3);
    SAVEDv(1, index) = V(1);
    SAVEDv(2, index) = V(2);
    SAVEDv(3, index) = V(3);
end


%% plots
% data_gpsP(any(isnan(data_gpsP),2),:) = []; 

figure;hold on;
plot(SAVEDp(1,:), SAVEDp(2,:))
plot(data_gpsP(1:data_length,1), data_gpsP(1:data_length,2), 'o')

figure;hold on;
plot(SAVEDp(1,:))
plot(data_gpsP(1:data_length,1), 'o')

figure;hold on;
plot(SAVEDp(2,:))
plot(data_gpsP(1:data_length,2), 'o')

figure;hold on;
plot(SAVEDp(3,:))
plot(data_gpsP(1:data_length,3), 'o')

figure;hold on;
plot(SAVEDv(1,:))
plot(data_gpsV(1:data_length,1), 'o')

figure;hold on;
plot(SAVEDv(2,:))
plot(data_gpsV(1:data_length,2), 'o')

figure;hold on;
plot(SAVEDv(3,:))
plot(data_gpsV(1:data_length,3), 'o')

