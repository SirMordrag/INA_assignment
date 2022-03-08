data_acc;
data_gyro;
data_gpsP;
data_gpsV;

% data convention:
% [x, y, z]
% [latitude, longitude, altitude]

index = 0;
T_sample = 1/100;

%% pseudo constants
g_n = getg_n( P(1), P(3) );
R_M = getR_M( P(1) );
R_N = getR_N( P(1) );

%% input data compensation
f = compensate_acc( data_acc(index) );
w = compensate_gyro( data_gyro(index) );

%% navigation equations
% update CDM
dDCM = DCM * [  0   -w(3)  w(2)
               w(3)   0   -w(1)
              -w(2)  w(1)   0  ];
DCM = DCM + dDCM * T_sample;
% update velocity
dV = DCM * f + g_n;
V = V + dV * T_sample;
% update position
dP = [ 1/(R_M + P(3))                    0                     0
           0               1/(cos(P(1)) * (R_N + P(3)))        0
           0                             0                    -1 ];
P = P + dP * T_sample;

