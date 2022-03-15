P = [1 1 1];
V = [1 2 3];
DCM = eye(3)
R_M = getR_M( P(1) )
R_N = getR_N( P(1) )

%% position
Fpp = [0                                           0                 (-V(1)/(R_M+P(3))^2)
       V(2)*sin(P(1))/((R_N+P(3))*cos(P(1))^2)     0                 -V(2)/((R_N+P(3))^2*cos(P(1)))
       0                                           0                  0]

Fpv = [1/(R_M+P(3))        0                           0
       0                   1/((R_N+P(3))*cos(P(1)))    0
       0                   0                          -1]

Fpr = O;

%% velocity

Fvp = [-2*Wn*V(2)-rn*V(2)/cos^2(P(1))               0   re*kd-rn*rd
       2*(Wn*V(1)+Wd*V(3))+(rn*V(1)/cos(P(1))^2)    0  -re*kd-rn*rd
       -2*V(2)*Wd                                   0   F63]


Fvv = [kd            2wd                 -re
      -(wd+Wd)       (kd-re*tan(P(1)))   wn+Wn
      2*re            -2*wn              0]


Fvr = [0    fd   -fe
      -fd   0     fn
       fe  -fn    0]   



%% attitude
Frp = [0                              0            (-V(2)/(R_N+P(3))^2)
       0                              0            (-V(1)/(R_M+P(3))^2)
       V(2)/((R_N+P(3))*cos(P(1))^2)  0            (-V(2)*tan(P(1)))/(R_N+P(3))^2]


Frv = [0               -1/(R_N+P(3))          0
       1/(R_M+P(3))     0                     0
       0               tan(P(1))/(R_N+P(3))   0]


Frr = [0    wd   -we
      -wd   0     wn
       we  -wn    0] 

%% model
I = eye(3)
O = zeros(3)
Fva = eye(3)
Frg = eye(3)
Faa = eye(3)
Fgg = eye(3)

F = [Fpp   Fpv  Fpr    O         O
     Fvp   Fvv  Fvr   -DCM*Fva   O
     Frp Fpv  Frr      O       DCM*Frg
     O     O    O      Faa       O
     O     O    O      O        Fgg]


Gama = [O   O   O   O
       -DCM O   O   O
        O   DCM O   O
        O   O   I   O
        O   O   O   I]