function [R_M, R_N, g_N] = get_pseudo_constants(Lat, Lon, Alt)
    % Get pseudo-constants (parameters which only need to be updated occasionally)

    % constants
    a = 6378137;
    e = 0.0818;
    a1 = 9.7803267714;
    a2 = 0.0052790414;
    a3 = 0.0000232718;
    a4 = -0.0000030876910891;
    a5 = 0.0000000043977311;
    a6 = 0.0000000000007211;

    % calculation
    R_M = a * (1 - e^2) / sqrt(1 - e^2 * (sin(Lat))^2)^3;
    R_N = a / sqrt(1 - e^2 * (sin(Lat))^2);
    g = a1*(1+a2*sin(Lat)^2+a3*sin(Lat)^4)+(a4+a5*sin(Lat)^2)*Alt+a6*Alt^2;
    g_N = [0 0 g].'; % in NED, transposed
end
