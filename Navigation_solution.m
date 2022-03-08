load("B(E)3(9)M38IMA_Data_Individual_Semester_Work.mat");
data_acc = [data(:,3), data(:,5), data(:,7)];
data_gyro = [data(:,11), data(:,13), data(:,15)];
data_gpsP = [data(:,20), data(:,21), data(:,22)];
data_gpsV = [data(:,28), data(:,29), data(:,27)];

% data convention:
% [x, y, z]
% [latitude, longitude, altitude]

index = 1;
T_sample = 1/200;

P = [49 14 500];
V = [0 0 0];
DCM = [1 0 0
       0 1 0
       0 0 1];

%% pseudo constants
g_n = getg_n( P(1), P(3) );
R_M = getR_M( P(1) );
R_N = getR_N( P(1) );

%% input data compensation
f = compensate_acc( data_acc(index,:) )
w = compensate_gyro( data_gyro(index,:) )

%% navigation equations
% update CDM
dDCM = DCM * [  0   -w(3)  w(2)
               w(3)   0   -w(1)
              -w(2)  w(1)   0  ]
DCM = DCM + dDCM * T_sample
% update velocity
f_n = DCM * f.'
dV = f_n.' + g_n
V = V + dV * T_sample
% update position
dP = [ 1/(R_M + P(3))                    0                     0
           0               1/(cos(P(1)) * (R_N + P(3)))        0
           0                             0                    -1 ] * V.'
P = P.' + dP * T_sample

