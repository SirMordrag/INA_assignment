a = 6378137
b = 6356752.3
e = sqrt(1-(b^2 / a^2))

% remove NaNs:
data_gpsP(any(isnan(data_gpsP),2),:) = []; 
plot(data_gpsP(1:4000,1), data_gpsP(1:4000,2))