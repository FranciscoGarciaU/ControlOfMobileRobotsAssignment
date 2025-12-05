function Assignment_Control_Methods()
% ASSIGNMENT_CONTROL_METHODS - Main script for differential drive robot control
% Compares KES (Kinematic Error Stabilization) and FL-PD (Feedback Linearization PD) controllers
%
% This script calls Main_control_code.m to run simulations and generates:
%   Figure 1: KES Controller performance
%   Figure 2: FL-PD Controller performance  
%   Figure 3: Controller comparison
%   Optional animations of each controller

clear all; close all; clc;

fprintf('=============================================\n');
fprintf('  DIFFERENTIAL DRIVE ROBOT CONTROL\n');
fprintf('    KES vs FL-PD CONTROLLER COMPARISON\n');
fprintf('=============================================\n\n');

%% RUN MAIN SIMULATION CODE
fprintf('Starting main simulation...\n');
fprintf('---------------------------\n');

% Call the main control code which runs both controllers
[qseq_KES, vwseq_KES, error_KES, qseq_FL, vwseq_FL, error_FL, xr, yr, t, Ts] = Main_control_code();

fprintf('\nMain simulation completed successfully!\n\n');

%% CREATE VISUALIZATIONS
fprintf('Generating visualizations...\n');
fprintf('---------------------------\n');

% Create individual controller figures
create_kes_figure(qseq_KES, vwseq_KES, error_KES, xr, yr, Ts);
create_flpd_figure(qseq_FL, vwseq_FL, error_FL, xr, yr, Ts);

% Create comparison figure
create_comparison_figure(qseq_KES, vwseq_KES, error_KES, qseq_FL, vwseq_FL, error_FL, xr, yr, Ts);

%% OPTIONAL: CREATE ANIMATIONS
create_animations = false;  % Set to false to skip animations
if create_animations
    fprintf('\nCreating controller animations...\n');
    fprintf('---------------------------------\n');
    
    % Create simple animation for KES controller
    animate_robot(qseq_KES, xr, yr, Ts, 'KES Controller');
    
    % Create simple animation for FL-PD controller  
    animate_robot(qseq_FL, xr, yr, Ts, 'FL-PD Controller');
    
    fprintf('Animations complete!\n');
end

%% FINAL SUMMARY
fprintf('\n=============================================\n');
fprintf('             SIMULATION COMPLETE\n');
fprintf('=============================================\n\n');

% Calculate performance metrics
rmse_KES = sqrt(mean(error_KES.^2));
rmse_FL = sqrt(mean(error_FL.^2));

fprintf('PERFORMANCE SUMMARY:\n');
fprintf('--------------------\n');
fprintf('KES Controller:\n');
fprintf('  • Final error: %.4f m\n', error_KES(end));
fprintf('  • RMSE: %.4f m\n', rmse_KES);
fprintf('  • Max error: %.4f m\n', max(error_KES));
fprintf('\nFL-PD Controller:\n');
fprintf('  • Final error: %.4f m\n', error_FL(end));
fprintf('  • RMSE: %.4f m\n', rmse_FL);
fprintf('  • Max error: %.4f m\n', max(error_FL));

% Calculate improvement
improvement = 100 * (rmse_KES - rmse_FL) / rmse_KES;
fprintf('\nFL-PD shows %.1f%% improvement in tracking accuracy.\n', improvement);

fprintf('\nFigures generated:\n');
fprintf('------------------\n');
fprintf('Figure 1: KES Controller performance\n');
fprintf('Figure 2: FL-PD Controller performance\n');
fprintf('Figure 3: Controller comparison\n');

if create_animations
    fprintf('\nAnimations displayed in separate figures.\n');
end

fprintf('\nTo re-run with different parameters, modify Main_control_code.m\n');
fprintf('=============================================\n');
end

%% VISUALIZATION FUNCTIONS

function create_kes_figure(qseq, vwseq, error_seq, xr, yr, Ts)
    % CREATE_KES_FIGURE - Creates Figure 1: KES Controller performance
    
    figure(1);
    clf;
    set(gcf, 'Position', [100, 100, 1200, 800], 'Name', 'KES Controller', 'NumberTitle', 'off');
    
    % Calculate metrics
    rmse_val = sqrt(mean(error_seq.^2));
    max_error = max(error_seq);
    final_error = error_seq(end);
    
    % 1. Trajectory tracking
    subplot(2, 2, 1);
    plot(xr, yr, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(qseq(1,:), qseq(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
    xlabel('X [m]'); ylabel('Y [m]');
    title('KES Controller - Trajectory Tracking');
    legend('Location', 'best');
    grid on; axis equal;
    
    
    % 2. Tracking error
    subplot(2, 2, 2);
    time_axis = 0:Ts:((length(error_seq)-1)*Ts);
    plot(time_axis, error_seq, 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Position Error [m]');
    title('Tracking Error');
    grid on;
    ylim([0, max(error_seq)*1.1]);
    
    % 3. Linear velocity
    subplot(2, 2, 3);
    plot(time_axis(1:size(vwseq,2)), vwseq(1,:), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('v [m/s]');
    title('Linear Velocity Command');
    grid on;
    
    % 4. Angular velocity
    subplot(2, 2, 4);
    plot(time_axis(1:size(vwseq,2)), vwseq(2,:), 'b-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('\omega [rad/s]');
    title('Angular Velocity Command');
    grid on;
    
    sgtitle('KES Controller Performance', 'FontSize', 14, 'FontWeight', 'bold');
end

function create_flpd_figure(qseq, vwseq, error_seq, xr, yr, Ts)
    % CREATE_FLPD_FIGURE - Creates Figure 2: FL-PD Controller performance
    
    figure(2);
    clf;
    set(gcf, 'Position', [100, 100, 1200, 800], 'Name', 'FL-PD Controller', 'NumberTitle', 'off');
    
    % Calculate metrics
    rmse_val = sqrt(mean(error_seq.^2));
    max_error = max(error_seq);
    final_error = error_seq(end);
    
    % 1. Trajectory tracking
    subplot(2, 2, 1);
    plot(xr, yr, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(qseq(1,:), qseq(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
    xlabel('X [m]'); ylabel('Y [m]');
    title('FL-PD Controller - Trajectory Tracking');
    legend('Location', 'best');
    grid on; axis equal;
        
    % 2. Tracking error
    subplot(2, 2, 2);
    time_axis = 0:Ts:((length(error_seq)-1)*Ts);
    plot(time_axis, error_seq, 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Position Error [m]');
    title('Tracking Error');
    grid on;
    ylim([0, max(error_seq)*1.1]);
    
    % 3. Linear velocity
    subplot(2, 2, 3);
    plot(time_axis(1:size(vwseq,2)), vwseq(1,:), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('v [m/s]');
    title('Linear Velocity Command');
    grid on;
    
    % 4. Angular velocity
    subplot(2, 2, 4);
    plot(time_axis(1:size(vwseq,2)), vwseq(2,:), 'r-', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('\omega [rad/s]');
    title('Angular Velocity Command');
    grid on;
    
    sgtitle('FL-PD Controller Performance', 'FontSize', 14, 'FontWeight', 'bold');
end

function create_comparison_figure(qseq_KES, vwseq_KES, error_KES, qseq_FL, vwseq_FL, error_FL, xr, yr, Ts)
    % CREATE_COMPARISON_FIGURE - Creates Figure 3: Controller comparison
    
    figure(3);
    clf;
    set(gcf, 'Position', [100, 100, 1400, 900], 'Name', 'Controller Comparison', 'NumberTitle', 'off');
    
    % Calculate metrics
    rmse_KES = sqrt(mean(error_KES.^2));
    rmse_FL = sqrt(mean(error_FL.^2));
    max_KES = max(error_KES);
    max_FL = max(error_FL);
    final_KES = error_KES(end);
    final_FL = error_FL(end);
    
    % 1. Trajectory comparison
    subplot(2, 3, 1);
    plot(xr, yr, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference');
    hold on;
    plot(qseq_KES(1,:), qseq_KES(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'KES');
    plot(qseq_FL(1,:), qseq_FL(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
    xlabel('X [m]'); ylabel('Y [m]');
    title('Trajectory Tracking Comparison');
    legend('Location', 'best');
    grid on; axis equal;
    
    % 2. Error comparison
    subplot(2, 3, 2);
    time_kes = 0:Ts:((length(error_KES)-1)*Ts);
    time_fl = 0:Ts:((length(error_FL)-1)*Ts);
    plot(time_kes, error_KES, 'b-', 'LineWidth', 1.5, 'DisplayName', 'KES');
    hold on;
    plot(time_fl, error_FL, 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
    xlabel('Time [s]'); ylabel('Position Error [m]');
    title('Tracking Error Comparison');
    legend('Location', 'best');
    grid on;
    
    % 3. Performance metrics bar chart
    subplot(2, 3, 3);
    metrics = {'RMSE', 'Max Error', 'Final Error'};
    kes_data = [rmse_KES; max_KES; final_KES];
    fl_data = [rmse_FL; max_FL; final_FL];
    
    bar([1:3], [kes_data, fl_data]);
    set(gca, 'XTickLabel', metrics);
    ylabel('Error [m]');
    title('Performance Metrics Comparison');
    legend('KES', 'FL-PD', 'Location', 'best');
    grid on;
    
    % Add value labels
    for i = 1:3
        text(i-0.18, kes_data(i)+0.001, sprintf('%.3f', kes_data(i)), 'FontSize', 9, 'Color', 'b');
        text(i+0.05, fl_data(i)+0.001, sprintf('%.3f', fl_data(i)), 'FontSize', 9, 'Color', 'r');
    end
    
    % 4. Linear velocity comparison
    subplot(2, 3, 4);
    plot(time_kes(1:size(vwseq_KES,2)), vwseq_KES(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'KES');
    hold on;
    plot(time_fl(1:size(vwseq_FL,2)), vwseq_FL(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
    xlabel('Time [s]'); ylabel('v [m/s]');
    title('Linear Velocity Comparison');
    legend('Location', 'best');
    grid on;
    
    % 5. Angular velocity comparison
    subplot(2, 3, 5);
    plot(time_kes(1:size(vwseq_KES,2)), vwseq_KES(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'KES');
    hold on;
    plot(time_fl(1:size(vwseq_FL,2)), vwseq_FL(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
    xlabel('Time [s]'); ylabel('\omega [rad/s]');
    title('Angular Velocity Comparison');
    legend('Location', 'best');
    grid on;
    
    % 6. Text summary
    subplot(2, 3, 6);
    axis off;
    
    % Calculate improvements
    rmse_improvement = 100 * (rmse_KES - rmse_FL) / rmse_KES;
    max_improvement = 100 * (max_KES - max_FL) / max_KES;
    final_improvement = 100 * (final_KES - final_FL) / final_KES;
    
    summary_text = {
        'CONTROLLER COMPARISON SUMMARY';
        '';
        'KES Controller:';
        sprintf('  • RMSE: %.4f m', rmse_KES);
        sprintf('  • Max Error: %.4f m', max_KES);
        sprintf('  • Final Error: %.4f m', final_KES);
        '';
        'FL-PD Controller:';
        sprintf('  • RMSE: %.4f m', rmse_FL);
        sprintf('  • Max Error: %.4f m', max_FL);
        sprintf('  • Final Error: %.4f m', final_FL);
        '';
        'IMPROVEMENT:';
        sprintf('  • RMSE: %.1f%% better', rmse_improvement);
        sprintf('  • Max Error: %.1f%% better', max_improvement);
        sprintf('  • Final Error: %.1f%% better', final_improvement);
        '';
    };
    
    text(0.05, 0.5, summary_text, 'VerticalAlignment', 'middle', ...
        'FontSize', 10, 'FontName', 'FixedWidth', 'Interpreter', 'none');
    
    sgtitle('Controller Performance Comparison: KES vs FL-PD', 'FontSize', 14, 'FontWeight', 'bold');
end

function animate_robot(qseq, xr, yr, Ts, controller_name)
    % ANIMATE_ROBOT - Creates simple animation of robot tracking
    
    fprintf('  Animating %s...\n', controller_name);
    
    % Create figure
    fig = figure('Position', [200, 200, 800, 600], ...
        'Name', sprintf('%s Animation', controller_name), ...
        'NumberTitle', 'off');
    
    % Determine plot limits
    all_x = [qseq(1,:), xr];
    all_y = [qseq(2,:), yr];
    
    x_min = min(all_x) - 0.5;
    x_max = max(all_x) + 0.5;
    y_min = min(all_y) - 0.5;
    y_max = max(all_y) + 0.5;
    
    % Ensure valid limits
    if x_min >= x_max
        x_min = -2; x_max = 2;
    end
    if y_min >= y_max
        y_min = -2; y_max = 2;
    end
    
    % Animation loop (limited to 50 frames for speed)
    n_frames = min(50, size(qseq,2));
    step_size = max(1, floor(size(qseq,2)/n_frames));
    
    for k = 1:step_size:size(qseq,2)
        clf;
        
        % Plot reference trajectory
        plot(xr, yr, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        hold on;
        
        % Plot robot path
        plot(qseq(1,1:k), qseq(2,1:k), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
        
        % Current robot state
        x = qseq(1,k);
        y = qseq(2,k);
        theta = qseq(3,k);
        
        % Draw robot as triangle
        robot_size = 0.15;
        robot_shape = robot_size * [
            cos(theta), sin(theta);
            cos(theta + 2*pi/3), sin(theta + 2*pi/3);
            cos(theta + 4*pi/3), sin(theta + 4*pi/3)
        ];
        robot_shape = robot_shape + [x, y];
        
        fill(robot_shape(:,1), robot_shape(:,2), 'b', ...
            'FaceAlpha', 0.6, 'EdgeColor', 'b', 'LineWidth', 2);
        
        % Draw orientation arrow
        quiver(x, y, 0.2*cos(theta), 0.2*sin(theta), 'r', ...
            'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % Current reference point
        ref_idx = min(k, length(xr));
        plot(xr(ref_idx), yr(ref_idx), 'ro', ...
            'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', 'Reference Point');
        
        % Labels and formatting
        xlabel('X [m]'); ylabel('Y [m]');
        title(sprintf('%s - Time: %.1f s', controller_name, (k-1)*Ts));
        legend('Location', 'best');
        grid on;
        axis equal;
        axis([x_min, x_max, y_min, y_max]);
        
        drawnow;
        pause(0.05);
    end
    
    fprintf('  %s animation complete.\n', controller_name);
end