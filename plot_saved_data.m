function [] = plot_saved_data(PLOT_DATA, INPUT_DATA)
    
    % NOTE: in case program takes too long to run, switch this to true
    is_quick_plotting = true;

    %% Unpack data 
    data_length = size(PLOT_DATA);
    data_length = data_length(1);
    
    data_acc  = [INPUT_DATA(:,3),  INPUT_DATA(:,5),  INPUT_DATA(:,7)];
    data_gyro = [INPUT_DATA(:,11), INPUT_DATA(:,13), INPUT_DATA(:,15)];
    data_gpsP = [INPUT_DATA(:,20), INPUT_DATA(:,21), INPUT_DATA(:,22)];
    data_gpsV = [INPUT_DATA(:,28), INPUT_DATA(:,29), INPUT_DATA(:,27)];
    clear INPUT_DATA;

    position = PLOT_DATA(:,1:3);
    velocity = PLOT_DATA(:,4:6);
    saved_DCM = PLOT_DATA(:,7:15);
    innovations = PLOT_DATA(:,16:21);
    P_matrix_diag = PLOT_DATA(:,22:36);
    K_matrix_arr = PLOT_DATA(:,37:126);
    state_vector = PLOT_DATA(:,127:141);
    clear PLOT_DATA;

    % process, reshape, convert
    attitude = zeros(data_length, 3);
    K_matrix = zeros(data_length, 15, 6);
    fprintf("Processing data for plot:     ");
    for i = 1:data_length
      if mod(i, 1000) == 0
        fprintf("\b\b\b\b%3.0f%%", 100 * i/data_length);
      end
      % attitude angles
      if ~is_quick_plotting
          [yaw, roll, pitch] = dcm2angle(reshape(saved_DCM(i,:), [3, 3]), "ZYX");
          attitude(i,:) = rad2deg([pitch; roll; yaw]);
      end
      % Kalman Gain
      K_matrix(i,:,:) = reshape(K_matrix_arr(i,:), 15, 6);
    end
    clear saved_DCM;
    fprintf("\nPlotting\n");
    
    %% Plot
    t = mkdir('Figures');

    figure;hold on;
    t = title("XY Position");
    plot(position(:,1), position(:,2))
    plot(data_gpsP(1:data_length,1), data_gpsP(1:data_length,2), 'o')
    saveas(gcf, ['Figures/', t.String, '.png'])
    legend('Estimate', 'GPS only')

    figure;hold on;
    t = title("XY Position (est)");
    plot(position(:,1), position(:,2))
    saveas(gcf, ['Figures/', t.String, '.png'])

    figure;hold on;
    t = title("XY Position (GPS)");
    plot(data_gpsP(1:data_length,1), data_gpsP(1:data_length,2), 'o')
    saveas(gcf, ['Figures/', t.String, '.png'])

    figure;hold on;
    t = title("X position");
    plot(position(:,1))
    plot(data_gpsP(1:data_length,1), 'o')
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', t.String, '.png'])
    
    figure;hold on;
    t = title("Y position");
    plot(position(:,2))
    plot(data_gpsP(1:data_length,2), 'o')
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', t.String, '.png'])
    
    figure;hold on;
    t = title("Z position");
    plot(position(:,3))
    plot(data_gpsP(1:data_length,3), 'o')
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', t.String, '.png'])
    
    figure;hold on;
    t = title("X velocity");
    plot(velocity(:,1))
    plot(data_gpsV(1:data_length,1), 'o')
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', t.String, '.png'])
    
    figure;hold on;
    t = title("Y velocity");
    plot(velocity(:,2))
    plot(data_gpsV(1:data_length,2), 'o')
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', t.String, '.png'])
    
    figure;hold on;
    t = title("Z velocity");
    plot(velocity(:,3))
    plot(data_gpsV(1:data_length,3), 'o')
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', t.String, '.png'])
    
    if ~is_quick_plotting
        figure;hold on;
        t = title("Angles (Estimate only)");
        plot(attitude)
        legend('Pitch', 'Roll', 'Yaw')
        saveas(gcf, ['Figures/', t.String, '.png'])
    end

    figure;hold on;
    t = title("Innovations");
    plot(innovations)
    legend('dPx','dPy','dPz','dVx','dVy','dVz')
    saveas(gcf, ['Figures/', t.String, '.png'])

    figure;hold on;
    t = title("P matrix");
    plot(P_matrix_diag)
    legend()
    saveas(gcf, ['Figures/', t.String, '.png'])

    figure;hold on;
    t = title("K matrix");
    plot(K_matrix_arr)
    legend()
    saveas(gcf, ['Figures/', t.String, '.png'])

    figure;hold on;
    t = title("State Vector");
    plot(state_vector)
    legend('dPx','dPy','dPz','dVx','dVy','dVz', 'rx', 'ry', 'rz', 'b_acc_x', 'b_acc_y', 'b_acc_z', 'b_gyro_x', 'b_gyro_y', 'b_gyro_z')
    saveas(gcf, ['Figures/', t.String, '.png'])

end

