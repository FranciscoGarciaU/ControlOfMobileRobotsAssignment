function animate_robot(qseq, xr, yr, Ts, controller_name, save_video)
    % ANIMATE_ROBOT - Creates animation of robot tracking
    % Inputs:
    %   qseq - robot state sequence [3 x N]
    %   xr, yr - reference trajectory
    %   Ts - sampling time
    %   controller_name - for title
    %   save_video - true to save as MP4
    
    fprintf('Creating animation for %s...\n', controller_name);
    
    figure('Position', [100, 100, 1200, 800], 'Name', sprintf('%s Animation', controller_name));
    
    % Create video writer if needed
    if save_video
        video_filename = sprintf('%s_animation.mp4', strrep(controller_name, ' ', '_'));
        writerObj = VideoWriter(video_filename, 'MPEG-4');
        writerObj.FrameRate = 10;
        open(writerObj);
        fprintf('Saving animation to: %s\n', video_filename);
    end
    
    % Determine bounds - FIXED: Ensure x_min < x_max
    all_x = [qseq(1,:), xr];
    all_y = [qseq(2,:), yr];
    
    x_min = min(all_x) - 0.5;
    x_max = max(all_x) + 0.5;
    y_min = min(all_y) - 0.5;
    y_max = max(all_y) + 0.5;
    
    % Ensure valid bounds
    if x_min >= x_max
        x_min = -2; x_max = 2;
    end
    if y_min >= y_max
        y_min = -2; y_max = 2;
    end
    
    % Animation loop
    n_steps = min(length(xr), size(qseq,2));
    if n_steps == 0
        fprintf('No data to animate!\n');
        return;
    end
    
    for k = 1:n_steps
        clf;
        
        % Main trajectory plot
        subplot(2, 2, [1, 3]);
        hold on;
        
        % Reference trajectory
        plot(xr, yr, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
        
        % Robot path so far
        plot(qseq(1,1:k), qseq(2,1:k), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
        
        % Current robot position and orientation
        x = qseq(1,k);
        y = qseq(2,k);
        theta = qseq(3,k);
        
        % Draw robot as triangle (if we have valid data)
        if ~isnan(x) && ~isnan(y) && ~isnan(theta)
            robot_size = 0.2;
            robot_shape = robot_size * [cos(theta), sin(theta);
                                        cos(theta + 2*pi/3), sin(theta + 2*pi/3);
                                        cos(theta + 4*pi/3), sin(theta + 4*pi/3)];
            robot_shape = robot_shape + [x, y];
            fill(robot_shape(:,1), robot_shape(:,2), 'b', 'FaceAlpha', 0.6, 'EdgeColor', 'b', 'LineWidth', 2);
            
            % Draw orientation arrow
            quiver(x, y, 0.3*cos(theta), 0.3*sin(theta), 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
            
            % Current reference point
            if k <= length(xr)
                plot(xr(k), yr(k), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Current Reference');
                
                % Draw error line
                plot([x, xr(k)], [y, yr(k)], 'g-', 'LineWidth', 1.5, 'DisplayName', 'Tracking Error');
            end
        end
        
        xlabel('X [m]'); ylabel('Y [m]');
        title(sprintf('%s - Time: %.1f s', controller_name, (k-1)*Ts));
        legend('Location', 'best'); grid on; axis equal;
        axis([x_min, x_max, y_min, y_max]);
        
        % Error plot
        subplot(2, 2, 2);
        if k <= length(xr) && k <= size(qseq,2)
            error_vals = sqrt((qseq(1,1:k) - xr(1:k)).^2 + (qseq(2,1:k) - yr(1:k)).^2);
            plot((0:k-1)*Ts, error_vals, 'b-', 'LineWidth', 1.5);
            hold on;
            plot((k-1)*Ts, error_vals(end), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
            
            % Add reference lines for error thresholds
            plot([0, (k-1)*Ts], [0.1, 0.1], 'r--', 'LineWidth', 1);
            plot([0, (k-1)*Ts], [0.05, 0.05], 'g--', 'LineWidth', 1);
            
            if k == 1
                legend('Error', 'Current Error', '0.1m threshold', '0.05m threshold');
            end
        end
        
        xlabel('Time [s]'); ylabel('Position Error [m]');
        title('Tracking Error Over Time');
        grid on;
        if exist('error_vals', 'var') && ~isempty(error_vals)
            ylim([0, max(1, max(error_vals)*1.1)]); % Ensure ylim is valid
        else
            ylim([0, 1]);
        end
        
        % Current status display
        subplot(2, 2, 4);
        axis off;
        
        if k <= length(xr) && k <= size(qseq,2)
            error_vals = sqrt((qseq(1,1:k) - xr(1:k)).^2 + (qseq(2,1:k) - yr(1:k)).^2);
            text_str = {
                sprintf('Current Status:');
                sprintf('Time: %.2f s', (k-1)*Ts);
                sprintf('Position: (%.3f, %.3f)', x, y);
                sprintf('Orientation: %.1f°', rad2deg(theta));
                sprintf('Error: %.3f m', error_vals(end));
                sprintf('Reference: (%.3f, %.3f)', xr(k), yr(k));
                '';
                'Performance Summary:';
                sprintf('Max Error: %.3f m', max(error_vals));
                sprintf('Avg Error: %.3f m', mean(error_vals));
                sprintf('Completion: %.1f%%', (k/n_steps)*100);
            };
        else
            text_str = {
                sprintf('Current Status:');
                sprintf('Time: %.2f s', (k-1)*Ts);
                sprintf('Position: (%.3f, %.3f)', x, y);
                sprintf('Orientation: %.1f°', rad2deg(theta));
                sprintf('Completion: %.1f%%', (k/n_steps)*100);
            };
        end
        text(0.1, 0.5, text_str, 'VerticalAlignment', 'middle', 'FontSize', 10);
        
        drawnow;
        
        % Save frame if requested
        if save_video
            try
                frame = getframe(gcf);
                writeVideo(writerObj, frame);
            catch ME
                fprintf('Warning: Could not capture frame %d: %s\n', k, ME.message);
            end
        end
        
        % Pause for real-time visualization
        pause(0.01);
    end
    
    % Close video writer
    if save_video
        try
            close(writerObj);
            fprintf('Animation saved successfully as %s!\n', video_filename);
        catch ME
            fprintf('Warning: Could not save video: %s\n', ME.message);
        end
    end
    
    fprintf('Animation complete.\n');
end