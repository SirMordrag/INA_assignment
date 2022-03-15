function [] = plot_saved_data(SAVED_DATA, data_gpsP, data_gpsV)
    data_length = size(SAVED_DATA);
    data_length = data_length(1);

    figure;hold on;
    title("XY Position")
    plot(SAVED_DATA(:,1), SAVED_DATA(:,2))
    plot(data_gpsP(1:data_length,1), data_gpsP(1:data_length,2), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("X position")
    plot(SAVED_DATA(:,1))
    plot(data_gpsP(1:data_length,1), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("Y position")
    plot(SAVED_DATA(:,2))
    plot(data_gpsP(1:data_length,2), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("Z position")
    plot(SAVED_DATA(:,3))
    plot(data_gpsP(1:data_length,3), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("X velocity")
    plot(SAVED_DATA(:,4))
    plot(data_gpsV(1:data_length,1), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("Y velocity")
    plot(SAVED_DATA(:,5))
    plot(data_gpsV(1:data_length,2), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("Z velocity")
    plot(SAVED_DATA(:,6))
    plot(data_gpsV(1:data_length,3), 'o')
    legend('IMU', 'GPS')
    
    figure;hold on;
    title("Angles (IMU only)")
    plot(SAVED_DATA(:,7))
    plot(SAVED_DATA(:,8))
    plot(SAVED_DATA(:,9))
    legend('Pitch', 'Roll', 'Yaw')



end