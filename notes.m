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

SAVED


%% NOTES
% earth rate is ommitted (Wn, We = 0)