function R_N = getR_M(Phi)
    
a = 6378137;
e = 0.0818;

R_N = a * (1 - e^2) / sqrt(1 - e^2 * (sin(Phi))^2)^3;

end