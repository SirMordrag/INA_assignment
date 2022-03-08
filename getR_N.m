function R_N = getR_N(Phi)
    
a = 6378137;
e = 0.0818;

R_N = a / sqrt(1 - e^2 * (sin(Phi))^2);

end