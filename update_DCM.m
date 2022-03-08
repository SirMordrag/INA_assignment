function R_new = update_DCM(R_old, w)
% R_old is old DCM matrix
% r_new is updated DCM matrix
% w is current vector of omegas as outputted by ARS

R_new = R_old + R_old * [  0   -w(3)  w(2)
                          w(3)   0   -w(1)
                         -w(2)  w(1)   0  ] * T_s;

end