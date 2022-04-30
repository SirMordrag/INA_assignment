function [] = plot_saved_data(SAVED_DATA, INPUT_DATA)
    
    % NOTE: in case program takes too long to run, switch this to true
    is_quick_plotting = false;

    %% Unpack data 
    data_length = size(SAVED_DATA);
    data_length = data_length(1);
    
    data_acc  = [INPUT_DATA(:,3),  INPUT_DATA(:,5),  INPUT_DATA(:,7)];
    data_gyro = [INPUT_DATA(:,11), INPUT_DATA(:,13), INPUT_DATA(:,15)];
    data_gpsP = [INPUT_DATA(:,20), INPUT_DATA(:,21), INPUT_DATA(:,22)];
    data_gpsV = [INPUT_DATA(:,28), INPUT_DATA(:,29), INPUT_DATA(:,27)];
    clear INPUT_DATA;

    position = SAVED_DATA(:,1:3);
    velocity = SAVED_DATA(:,4:6);
    saved_DCM = SAVED_DATA(:,7:15);
    innovations = SAVED_DATA(:,16:21);
    P_matrix_diag = SAVED_DATA(:,22:36);
    K_matrix_arr = SAVED_DATA(:,37:126);
    state_vector = SAVED_DATA(:,127:141);
    bias_acc = SAVED_DATA(:,142:144);
    bias_gyro = SAVED_DATA(:,145:147);
    clear SAVED_DATA;

    %% Plot
    t = mkdir('Figures');

    % XY Position
    figure;
    subplot(1,3,1);
    hold on;
    plot(position(:,1), position(:,2))
    plot(data_gpsP(1:data_length,1), data_gpsP(1:data_length,2), '.')
    title("XY Position overlay");
    legend('Estimate', 'GPS only')
    subplot(1,3,2);
    plot(position(:,1), position(:,2))
    title("XY Position (est)");
    subplot(1,3,3);
    plot(data_gpsP(1:data_length,1), data_gpsP(1:data_length,2), '.')
    title("XY Position (GPS)");
    saveas(gcf, ['Figures/', 'XY position', '.png'])

    % Position Components
    figure;
    subplot(1,3,1);hold on;
    plot(position(:,1))
    plot(data_gpsP(1:data_length,1), '.')
    legend('Estimate', 'GPS only')
    title("X position");
    subplot(1,3,2);hold on;
    plot(position(:,2))
    plot(data_gpsP(1:data_length,2), '.')
    title("Y position");
    legend('Estimate', 'GPS only')
    subplot(1,3,3);hold on;
    plot(position(:,3))
    plot(data_gpsP(1:data_length,3), '.')
    title("Z position");
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', 'Position', '.png'])

    % Velocity Components
    figure;
    subplot(1,3,1);hold on;
    plot(velocity(:,1))
    plot(data_gpsV(1:data_length,1), '.')
    legend('Estimate', 'GPS only')
    title("X velocity");
    subplot(1,3,2);hold on;
    plot(velocity(:,2))
    plot(data_gpsV(1:data_length,2), '.')
    title("Y velocity");
    legend('Estimate', 'GPS only')
    subplot(1,3,3);hold on;
    plot(velocity(:,3))
    plot(data_gpsV(1:data_length,3), '.')
    title("Z velocity");
    legend('Estimate', 'GPS only')
    saveas(gcf, ['Figures/', 'Velocity', '.png'])

    % Innovations
    figure;
    subplot(1,2,1); hold on;
    plot(innovations(:,1:3))
    title("Innovations: dP");
    legend('dPx','dPy','dPz')
    subplot(1,2,2); hold on;
    plot(innovations(:,4:6))
    title("Innovations: dV");
    legend('dVx','dVy','dVz')
    saveas(gcf, ['Figures/', 'Innovations', '.png'])

    % P matrix
    figure;
    subplot(2,3,1);hold on;
    plot(P_matrix_diag(:,1:3))
    title("P matrix: P");
    legend('dPx','dPy','dPz')
    subplot(2,3,2);hold on;
    plot(P_matrix_diag(:,4:6))
    title("P matrix: V");
    legend('dVx','dVy','dVz')
    subplot(2,3,3);hold on;
    plot(P_matrix_diag(:,7:9))
    title("P matrix: r");
    legend('rx', 'ry', 'rz')
    subplot(2,3,4);hold on;
    plot(P_matrix_diag(:,10:12))
    title("P matrix: bacc");
    legend('b_acc_x', 'b_acc_y', 'b_acc_z')
    subplot(2,3,5);hold on;
    plot(P_matrix_diag(:,13:15))
    title("P matrix: bgyro");
    legend('b_gyro_x', 'b_gyro_y', 'b_gyro_z')
    saveas(gcf, ['Figures/', 'P matrix', '.png'])

    % K matrix
    figure;hold on;
    subplot(3,2,1);
    plot(K_matrix_arr(:,1:15))
    title("K Matrix - Row 1");
    subplot(3,2,2);
    plot(K_matrix_arr(:,16:30))
    title("K Matrix - Row 2");
    subplot(3,2,3);
    plot(K_matrix_arr(:,31:45))
    title("K Matrix - Row 3");
    subplot(3,2,4);
    plot(K_matrix_arr(:,[46:53,55:60])) % skip yaw element (invalid)
    title("K Matrix - Row 4");
    subplot(3,2,5);
    plot(K_matrix_arr(:,[61:68,70:75])) % skip yaw element (invalid)
    title("K Matrix - Row 5");
    subplot(3,2,6);
    plot(K_matrix_arr(:,76:90))
    title("K Matrix - Row 6");
    saveas(gcf, ['Figures/', 'K matrix', '.png'])

    % State Vector
    figure;
    subplot(2,3,1);hold on;
    plot(state_vector(:,1:3))
    title("State Vector: dP");
    legend('dPx','dPy','dPz')
    subplot(2,3,2);hold on;
    plot(state_vector(:,4:6))
    title("State Vector: dV");
    legend('dVx','dVy','dVz')
    subplot(2,3,3);hold on;
    plot(state_vector(:,7:9))
    title("State Vector: dr");
    legend('rx', 'ry', 'rz')
    subplot(2,3,4);hold on;
    plot(state_vector(:,10:12))
    title("State Vector: dbacc");
    legend('b_acc_x', 'b_acc_y', 'b_acc_z')
    subplot(2,3,5);hold on;
    plot(state_vector(:,13:15))
    title("State Vector: dbgyro");
    legend('b_gyro_x', 'b_gyro_y', 'b_gyro_z')
    saveas(gcf, ['Figures/', 'State Vector', '.png'])

    % Biases
    figure;
    subplot(1,2,1);hold on;
    plot(bias_acc)
    title("Accelerometer Biases");
    legend('acc X', 'acc Y','acc Z')
    subplot(1,2,2);hold on;
    plot(bias_gyro)
    legend('gyro X','gyro Y','gyro Z')
    title("Gyro Biases")
    saveas(gcf, ['Figures/', 'Biases', '.png'])

    if is_quick_plotting
        return
    end

    %% angles
    attitude = zeros(data_length, 3);
    fprintf("Plotting angles, this will take a couple of minutes\n");
    fprintf("Note: If it takes too long, please set 'is_quick_plotting' to true in plot_saved_data function (line 4). Attitude will not be plotted.\n")
    fprintf("Processing data:     ");
    for i = 1:data_length
      if mod(i, 1000) == 0
        fprintf("\b\b\b\b%3.0f%%", 100 * i/data_length);
      end
      % attitude angles
      [yaw, roll, pitch] = dcm2angle(reshape(saved_DCM(i,:), [3, 3]), "ZYX");
      attitude(i,:) = rad2deg([pitch; roll; yaw]);
    end

    figure;hold on;
    t = title("Angles (Estimate only)");
    plot(attitude)
    legend('Pitch', 'Roll', 'Yaw')
    saveas(gcf, ['Figures/', t.String, '.png'])
end

