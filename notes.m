% a = 6378137
% b = 6356752.3
% e = sqrt(1-(b^2 / a^2))
% 
% % remove NaNs:
% data_gpsP(any(isnan(data_gpsP),2),:) = []; 
% plot(data_gpsP(1:4000,1), data_gpsP(1:4000,2))


SAVED = zeros(5, 9);
SAVED(1, 1:3) = [1, 1, 1];
lol = [1, 2, 3];
bol = [5, 7, 33];
gol = [9, 21, 355];
SAVED(2, 4:6) = lol;
SAVED(3,:) = [lol, bol, gol];

SAVED;

H_kf = [1 0 0 0 0 0 0 0 0 0 0 0
        0 1 0 0 0 0 0 0 0 0 0 0
        0 0 1 0 0 0 0 0 0 0 0 0
        0 0 0 1 0 0 0 0 0 0 0 0
        0 0 0 0 1 0 0 0 0 0 0 0
        0 0 0 0 0 1 0 0 0 0 0 0
    ];
dx_next = [1 2 3 4 5 6 7 8 9 10 11 12].';
H_kf * dx_next;

a = [1 2 3];
b = a .* eye(3);
c = 2 .* eye(3);
c * b;
c * b * c.';
eye(15,12) * eye(12,1);

%K_matrix = zeros(15, 6, data_length);
%     for i = 1:data_length
%         K_matrix(:,:,i) = reshape(K_matrix_arr(i,:), 15, 6)
%     end


syms p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34 p41 p42 p43 p44
syms r1 r2 r3
P_kf_predict = [p11 p12 p13 p14
                p21 p22 p23 p24
                p31 p32 p33 p34
                p41 p42 p43 p44];
H_kf = eye(3, 4);
R_kf = [r1 0 0
        0  r2 0
        0 0   r3];
K_kf = P_kf_predict * H_kf.' / (H_kf * P_kf_predict * H_kf.' + R_kf)
K_kf(1,1)
K_kf(2,2)
K_kf(3,3)

%                  .....                                       
% (p1*p5 - p2*p4 + p1*r2)/(p1*p5 - p2*p4 + p1*r2 + p5*r1 + r1*r2)
% (p1*p5 - p2*p4 + p5*r1)/(p1*p5 - p2*p4 + p1*r2 + p5*r1 + r1*r2)

%                                                                                      ..........   ..........   ..........   ..........   .........
% (p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + p11*p22*r3 - p12*p21*r3 + p11*p33*r2 - p13*p31*r2 + p11*r2*r3)/(p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + p11*p22*r3 - p12*p21*r3 + p11*p33*r2 - p13*p31*r2 + p22*p33*r1 - p23*p32*r1 + p11*r2*r3 + p22*r1*r3 + p33*r1*r2 + r1*r2*r3)
% (p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + p11*p22*r3 - p12*p21*r3 + p22*p33*r1 - p23*p32*r1 + p22*r1*r3)/(p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + p11*p22*r3 - p12*p21*r3 + p11*p33*r2 - p13*p31*r2 + p22*p33*r1 - p23*p32*r1 + p11*r2*r3 + p22*r1*r3 + p33*r1*r2 + r1*r2*r3)
% (p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + p11*p33*r2 - p13*p31*r2 + p22*p33*r1 - p23*p32*r1 + p33*r1*r2)/(p11*p22*p33 - p11*p23*p32 - p12*p21*p33 + p12*p23*p31 + p13*p21*p32 - p13*p22*p31 + p11*p22*r3 - p12*p21*r3 + p11*p33*r2 - p13*p31*r2 + p22*p33*r1 - p23*p32*r1 + p11*r2*r3 + p22*r1*r3 + p33*r1*r2 + r1*r2*r3)

%% NOTES
% earth rate is ommitted (Wn, We = 0)
% Multiplying an m×n matrix on the right by an n×p matrix yields an m×p matrix. 