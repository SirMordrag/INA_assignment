function gn = getg_n(lat,alt)
% Computation of gravitational acceleration in Earth's frame of reference based on WGS84 Gravity model
% Precision of the computation is 10^-6 up to the 20.000 m of height above surface
% INPUT:          latitude (rad); longitude (rad); height (m)
% OUTPUT:         gravitational acceleration (m/s^2)

% WGS84 Gravity model constants:
a1 = 9.7803267714;
a2 = 0.0052790414;
a3 = 0.0000232718;
a4 = -0.0000030876910891;
a5 = 0.0000000043977311;
a6 = 0.0000000000007211;
% Gravity computation

% The magnitude of the normal gravity vector over the surface of the ellipsoid can
% be computed as a function of latitude and height by a closed form expression
% known as the Somigliana formula (Schwarz and Wei Jan 1999):
g = a1*(1+a2*sin(lat)^2+a3*sin(lat)^4)+(a4+a5*sin(lat)^2)*alt+a6*alt^2;    % source - Mathematics in navigation (2.123)
gn = [0 0 g];

end
