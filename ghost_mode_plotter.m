%% Ghost Mode PID Tuning Plotter
% Optimized version with high-speed batch reading and immediate START signal.

clear; close all; clc;

%% Configuration
portName = 'COM15'; % Update this to your STM32's COM port
baudRate = 115200;
timeout = 0.1;

%% Initialize Data Storage
current_data = [];
previous_data_1 = []; 
previous_data_2 = []; 
history_settling = []; 
history_overshoot = [];
run_count = 0;
is_collecting = false;
start_pos = 0;
dir_mult = 1;

%% Setup Plot
fig = figure('Name', 'Ghost Mode PID Tuning (Normalized)', 'NumberTitle', 'off', 'Color', [0.1 0.1 0.1], 'Position', [100, 100, 1000, 800]);

% Main Position Plot
ax_main = subplot(2, 2, [1, 2], 'Parent', fig, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
hold(ax_main, 'on'); grid(ax_main, 'on'); set(ax_main, 'GridColor', [0.4 0.4 0.4]);
xlabel(ax_main, 'Time (s)'); ylabel(ax_main, 'Relative Position (deg)');
ylim(ax_main, [0 200]); xlim(ax_main, [0 20]);
p_target = plot(ax_main, NaN, NaN, 'm--', 'LineWidth', 1.5, 'DisplayName', 'Target');
p_prev2 = plot(ax_main, NaN, NaN, 'Color', [1 1 0 0.15], 'LineWidth', 1.2, 'DisplayName', 'Prev Run -2');
p_prev1 = plot(ax_main, NaN, NaN, 'Color', [1 1 0 0.45], 'LineWidth', 1.5, 'DisplayName', 'Prev Run -1');
p_current = plot(ax_main, NaN, NaN, 'y', 'LineWidth', 2.0, 'DisplayName', 'Real-time');
legend(ax_main, 'TextColor', 'w', 'Location', 'northeast', 'Color', [0.2 0.2 0.2]);

% Settling Time Subplot
ax_settle = subplot(2, 2, 3, 'Parent', fig, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
hold(ax_settle, 'on'); grid(ax_settle, 'on'); set(ax_settle, 'GridColor', [0.3 0.3 0.3]);
ylabel(ax_settle, 'Settling Time (s)'); xlabel(ax_settle, 'Run #');
ylim(ax_settle, [0 20]);
p_settle_hist = plot(ax_settle, NaN, NaN, 'c-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'c');

% Overshoot Subplot
ax_over = subplot(2, 2, 4, 'Parent', fig, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w');
hold(ax_over, 'on'); grid(ax_over, 'on'); set(ax_over, 'GridColor', [0.3 0.3 0.3]);
ylabel(ax_over, 'Overshoot (%)'); xlabel(ax_over, 'Run #');
ylim(ax_over, [0 10]);
p_over_hist = plot(ax_over, NaN, NaN, 'g-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'g');

% Text Info Overlay
txt_info = annotation('textbox', [0.3, 0.01, 0.4, 0.05], 'String', 'Ready', 'Color', 'w', 'EdgeColor', 'none', 'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold');

%% Serial Communication
try
    s = serialport(portName, baudRate, 'Timeout', timeout);
    configureTerminator(s, "CR/LF"); flush(s);
    fprintf('Connected to %s. Waiting for START...\n', portName);
    
    while ishandle(fig)
        % OPTIMIZATION: Read all available lines in a single loop
        while s.NumBytesAvailable > 0
            line = readline(s);
            if isempty(line), continue; end
            
            if contains(line, "START")
                % Immediate response to Y-press
                parts = strsplit(line, ',');
                if length(parts) >= 2
                    target_val = str2double(parts{2});
                    set(p_target, 'XData', [0 20], 'YData', [target_val target_val]);
                end
                
                % Shift history
                if ~isempty(current_data)
                    previous_data_2 = previous_data_1;
                    previous_data_1 = current_data;
                    if ~isempty(previous_data_1)
                        set(p_prev1, 'XData', (previous_data_1(:,1)-previous_data_1(1,1))/1000, 'YData', previous_data_1(:,2));
                    end
                    if ~isempty(previous_data_2)
                        set(p_prev2, 'XData', (previous_data_2(:,1)-previous_data_2(1,1))/1000, 'YData', previous_data_2(:,2));
                    end
                end
                
                current_data = [];
                set(p_current, 'XData', NaN, 'YData', NaN); % Clear current plot
                is_collecting = true;
                start_pos = NaN; 
                set(txt_info, 'String', 'Motor Running...', 'Color', 'y');
                
            elseif contains(line, "END")
                if is_collecting
                    is_collecting = false;
                    % Process and Plot
                    if ~isempty(current_data)
                        t = (current_data(:,1) - current_data(1,1)) / 1000.0;
                        pos = current_data(:,2);
                        target = current_data(end,3);
                        
                        set(p_current, 'XData', t, 'YData', pos);
                        set(p_target, 'XData', [0 20], 'YData', [target target]);
                        
                        peak_val = max(pos);
                        if target > 1
                            overshoot_pct = max(0, ((peak_val - target)/target)*100);
                        else
                            overshoot_pct = 0;
                        end
                        
                        idx_outside = find(abs(pos - target) > 1.0, 1, 'last');
                        if isempty(idx_outside)
                            settle_time = 0;
                        else
                            settle_time = t(idx_outside);
                        end
                        
                        run_count = run_count + 1;
                        history_settling = [history_settling; [run_count, settle_time]];
                        history_overshoot = [history_overshoot; [run_count, overshoot_pct]];
                        
                        set(p_settle_hist, 'XData', history_settling(:,1), 'YData', history_settling(:,2));
                        set(p_over_hist, 'XData', history_overshoot(:,1), 'YData', history_overshoot(:,2));
                        set(txt_info, 'String', sprintf('Run #%d: Settling = %.2fs, Overshoot = %.1f%%', run_count, settle_time, overshoot_pct), 'Color', 'w');
                    end
                end
                
            elseif contains(line, "DATA")
                % Fast buffering
                try
                    parts = strsplit(line, ',');
                    tick = str2double(parts{2});
                    pos_raw = str2double(parts{3}) / 100.0;
                    target_raw = str2double(parts{4}) / 100.0;
                    
                    if isnan(start_pos)
                        start_pos = pos_raw;
                        dist = target_raw - start_pos;
                        if dist < 0, dir_mult = -1; else, dir_mult = 1; end
                    end
                    
                    current_data = [current_data; [tick, (pos_raw-start_pos)*dir_mult, (target_raw-start_pos)*dir_mult]];
                catch
                end
                
            elseif contains(line, "PREVIEW")
                if ~is_collecting
                    try
                        parts = strsplit(line, ',');
                        curr_abs = str2double(parts{2}) / 100.0;
                        ghost_target = str2double(parts{3}) / 100.0;
                        set(txt_info, 'String', sprintf('Next Target: %.1f deg', abs(ghost_target - curr_abs)), 'Color', 'm');
                    catch
                    end
                end
            end
        end
        drawnow limitrate;
        pause(0.0001);
    end
catch ME
    fprintf('Error: %s\n', ME.message);
    if exist('s', 'var'), clear s; end
end
clear s;
