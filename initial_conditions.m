function [P, V, DCM] = initial_conditions(data_acc, data_gpsP, data_gpsV, init_time)
    % Get initial position, velocity and attitude
    % assuming steady conditions in init period, but some motion in direction of flight

    % init_time to samples
    init_acc_samples = init_time * 200;
    init_gps_samples = init_time * 5;

    % strip GPS data of NaN's
    data_gpsP(any(isnan(data_gpsP),2),:) = []; 
    data_gpsV(any(isnan(data_gpsV),2),:) = []; 
    
    % position and velocity from first GPS reading
    P = data_gpsP(1,:).';
    V = data_gpsV(1,:).';

    % pitch & roll from ACC
    f_mean = mean(data_acc(1:init_acc_samples,:));
    pitch = atan2(f_mean(1), f_mean(3)) - pi;
    roll  = atan2(f_mean(2), f_mean(3)) + pi;
    
    % yaw from linear regression of position in init
    P_fit = data_gpsP(1:init_gps_samples,1:end);
    yaw = P_fit(:,1)\P_fit(:,2);

    % DCM from angles
    DCM = angle2dcm(yaw, roll, pitch, "ZYX");
end
