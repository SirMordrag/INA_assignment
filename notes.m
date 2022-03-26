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
    for i = 1:data_length
        K_matrix(:,:,i) = reshape(K_matrix_arr(i,:), 15, 6)
    end

%% NOTES
% earth rate is ommitted (Wn, We = 0)
% Multiplying an m×n matrix on the right by an n×p matrix yields an m×p matrix. 