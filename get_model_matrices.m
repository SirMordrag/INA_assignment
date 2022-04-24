function [F, G] = get_model_matrices(P, V, DCM, f, w, r, R_M, R_N, T_gps)
    % gets F and G matrix for the model
    % inputs: position (vec), velocity (vec), DCM (mat), acc (vec), gyro (vec), rho (vec), kd (num), F63 (num), R_M (num), R_N (num), T_gps (num)
    I = eye(3);
    O = zeros(3);
    
    %% position
    Fpp = [ 0                                           0    -V(1)/(R_M+P(3))^2
            V(2)*sin(P(1))/((R_N+P(3))*cos(P(1))^2)     0    -V(2)/((R_N+P(3))^2*cos(P(1)))
            0                                           0     0];
    
    Fpv = [ 1/(R_M+P(3))     0                            0 
            0                1/((R_N+P(3))*cos(P(1)))     0
            0                0                           -1];
    
    Fpr = O;
    
    %% velocity
    kd = 0; % see page 397 of book, actually kd = vd / Re, where Re is Earth radius -> waaaaay to small -> omitted

    Fvp = [-r(1)*V(2)/cos(P(1))^2     0     r(2)*kd-r(1)*r(3)
            r(1)*V(1)/cos(P(1))^2     0    -r(2)*kd-r(1)*r(3)
            0                         0     r(1)^2 + r(2)^2];
    
    
    Fvv = [ kd         2*w(3)               -r(2)
           -w(3)       kd-r(2)*tan(P(1))     w(1)
            2*r(2)    -2*w(1)                0];
    
    
    Fvr = [ 0        f(3)    -f(2)
           -f(3)     0        f(1)
            f(2)    -f(1)     0];   
    
    %% attitude
    Frp = [ 0                                 0     V(2)/(R_N+P(3))^2
            0                                 0    -V(1)/(R_M+P(3))^2
            V(2)/((R_N+P(3))*cos(P(1))^2)     0    -V(2)*tan(P(1))/(R_N+P(3))^2 ];
    
    
    Frv = [ 0               -1/(R_N+P(3))             0
            1/(R_M+P(3))     0                        0
            0                tan(P(1))/(R_N+P(3))     0];
    
    
    Frr = [ 0        w(3)    -w(2)
           -w(3)     0        w(1)
            w(2)    -w(1)     0]; 
    
    %% uncertainties
    % dunno
    Fva = O;
    Frg = O;
    Faa = O;
    Fgg = O;
    
    %% model
    % continous F
    F_c   = [ Fpp     Fpv     Fpr     O           O
              Fvp     Fvv     Fvr    -DCM*Fva     O
              Frp     Frv     Frr     O           DCM*Frg
              O       O       O       Faa         O
              O       O       O       O           Fgg];
    % discreet F
    F = eye(15) + F_c * T_gps + 0.5 * T_gps^2 * F_c^2;
    
    % WARNING element (1,1) is probably missing something (C)
    % discreet G
    G = [-0.5 * I * T_gps^2     O               O     O
         -DCM * T_gps           O               O     O
          O                     DCM * T_gps     O     O
          O                     O               I     O
          O                     O               O     I];

end