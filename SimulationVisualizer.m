classdef SimulationVisualizer < handle
    % SIMULATIONVISUALIZER Real-time 2D visualization for the simulation
    %
    % Syntax:
    %   viz = SimulationVisualizer()
    %
    % Description:
    %   Provides a live 2D visualization of agent and target states for the
    %   persistent monitoring simulation. Displays positions, uncertainty,
    %   battery levels, operating modes (color-coded), and global metrics.
    %
    % Example:
    %   viz = SimulationVisualizer();
    %   viz.initialize_plot([t1,t2], [a1]);
    %   viz.update_visualization([t1,t2], [a1], current_time);
    
    properties
        figure_handle
        target_plots
        agent_plots
        agent_orientation_plots
        uncertainty_text
        time_text
        xlim_range
        ylim_range
        target_uncertainty_texts
        target_objective_texts
        target_global_objective_texts
        agent_battery_texts
        agent_energy_texts
        agent_index_texts
        blink_state = false  % Track blinking state
    end
    
    methods
        % SimulationVisualizer constructor
        %   viz = SimulationVisualizer()
        %
        % Creates a figure window and prepares visualization state.
        function obj = SimulationVisualizer()
            % Initialize figure and basic setup
            obj.figure_handle = figure('Name', 'Simulation Visualization', ...
                                     'NumberTitle', 'off', ...
                                     'Position', [100, 100, 800, 600]);
            hold on;
            grid on;
            xlabel('X Position');
            ylabel('Y Position');
            title('Dynamic Simulation');
            
            % Initialize arrays
            obj.target_plots = [];
            obj.agent_plots = [];
            obj.agent_orientation_plots = [];
            obj.target_uncertainty_texts = [];
            obj.xlim_range = [-5, 15];
            obj.ylim_range = [-5, 15];
        end
        
        % initialize_plot Prepare initial plot elements for targets and agents
        %   viz.initialize_plot(targets, agents)
        %
        % Inputs:
        %   targets - array of Target objects
        %   agents  - array of Agent objects
        %
        % Creates marker handles and labels for visualization.
        function initialize_plot(obj, targets, agents)
            cla; % Clear current axes
            
            % Plot targets
            for i = 1:length(targets)
                target = targets(i);
                pos = target.position;
                
                % Create target plot (circle)
                plot_handle = plot(pos(1), pos(2), 'ro', 'MarkerSize', 10, ...
                                 'MarkerFaceColor', 'red', 'LineWidth', 2);
                
                % Add target index text
                text(pos(1)+0.5, pos(2)+0.5, sprintf('T%d', target.index), ...
                     'FontSize', 10, 'FontWeight', 'bold');
                
                % Store plot handle
                obj.target_plots = [obj.target_plots, plot_handle];
            end
            
            % Plot agents 
            for i = 1:length(agents)
                agent = agents(i);
                pos = agent.position;
                x = pos(1);
                y = pos(2);
                if length(pos) >= 3
                    theta = pos(3);
                else
                    theta = 0;  % defalut orientation is 0
                end
                % Create orientation arrow
                arrow_lenth = 1.0;
                x_end = x + arrow_lenth * cos(theta);
                y_end = y + arrow_lenth * sin(theta);
                
                % 
                % Create agent plot (triangle)
                orientation_handle = plot([x, x_end], [y, y_end], 'k-', 'LineWidth', 2, 'Color', 'black');
                plot_handle = plot(x, y, '^', 'MarkerSize', 14, ...
                                 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black', 'LineWidth', 2);
                
                % Add agent index text
                % move this to update state that move with the agent 
                
                % Store plot handle
                obj.agent_plots = [obj.agent_plots, plot_handle];
                obj.agent_orientation_plots = [obj.agent_orientation_plots, orientation_handle];
            end
            
            % Set axis limits
            xlim(obj.xlim_range);
            ylim(obj.ylim_range);
            
            % Add legend with correct shapes
            legend_entries = {};
            legend_handles = [];
            
            if ~isempty(obj.target_plots)
                % Create a dummy plot for target legend (circle)
                h_target = plot(NaN, NaN, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
                legend_handles = [legend_handles, h_target];
                legend_entries{end+1} = 'Targets';
            end
            
            if ~isempty(obj.agent_plots)
                % Create a dummy plot for agent legend (triangle)
                h_agent = plot(NaN, NaN, '^', 'MarkerSize', 8, 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black');
                legend_handles = [legend_handles, h_agent];
                legend_entries{end+1} = 'Agents';
            end
            
            if ~isempty(legend_entries)
                legend(legend_handles, legend_entries, 'Location', 'northeast');
            end
        end
        
        % update_visualization Refresh the visualization for the current state
        %   viz.update_visualization(targets, agents, current_time)
        %
        % Inputs:
        %   targets      - array of Target objects
        %   agents       - array of Agent objects
        %   current_time - current simulation time (seconds)
        %
        % Updates positions, colors, metric text fields, and draws the figure.
        function update_visualization(obj, targets, agents, current_time)
            % Update all plots with current states
            
            % Update target plots
            for i = 1:length(targets)
                if i <= length(obj.target_plots)
                    target = targets(i);
                    pos = target.position;
                    
                    % Update target position
                    set(obj.target_plots(i), 'XData', pos(1), 'YData', pos(2));
                    
                    % Set color based on target mode
                    uncertainty = target.uncertainty;
                    objective = target.objective;

                    if target.mode == "monitor"
                        color = 'green';
                    else
                        color = 'red';
                    end
                    set(obj.target_plots(i), 'MarkerFaceColor', color);
                    
                    % Update uncertainty text near the target
                    if length(obj.target_uncertainty_texts) >= i && ~isempty(obj.target_uncertainty_texts(i))
                        try
                            delete(obj.target_uncertainty_texts(i));
                            delete(obj.target_objective_texts(i));
                        catch
                            % Text object might already be deleted
                        end
                    end
                    
                    obj.target_uncertainty_texts(i) = text(pos(1)+0.5, pos(2)-0.5, ...
                        sprintf('R=%.1f', uncertainty), ...
                        'FontSize', 9, 'Color', 'black', 'FontWeight', 'bold');
                    obj.target_objective_texts(i) = text(pos(1)+0.5, pos(2)-1.5, ...
                        sprintf('objective=%.1f', objective), ...
                        'FontSize', 9, 'Color', 'black', 'FontWeight', 'bold');
                end
            end
            
            % Update agent plots (if any)
            for i = 1:length(agents)
                if i <= length(obj.agent_plots)
                    agent = agents(i);
                    pos = agent.position;
                    x = pos(1);
                    y = pos(2);
                    if length(pos) >= 3
                        theta = pos(3);
                    else
                        theta = 0;  % defalut orientation is 0
                    end
                    % Update agent position
                    set(obj.agent_plots(i), 'XData', x, 'YData', y);
                    % Update agent orientation
                    arrow_lenth = 1.0;
                    x_end = x + arrow_lenth * cos(theta);
                    y_end = y + arrow_lenth * sin(theta);
                    set(obj.agent_orientation_plots(i), 'XData', [x, x_end], 'YData', [y, y_end]);

                    battery_percentage = agent.battery_percentage;
                    e_total = agent.e_total;

                    % set color based on agent mode
                    if agent.mode == "traveling"
                        color = 'blue';
                    elseif agent.mode == "dwelling"
                        color = 'cyan';
                    elseif agent.mode == "planning"
                        color = 'magenta';
                    elseif agent.mode == "power_outage"
                        % Toggle blink state each update
                        obj.blink_state = ~obj.blink_state;
                        if obj.blink_state
                            color = 'yellow';
                        else
                            color = 'red';
                        end
                    else
                        color = 'black';
                    end
                    set(obj.agent_plots(i), 'MarkerFaceColor', color);

                    % Update agent battery and energy text
                    if length(obj.agent_battery_texts) >= i && ~isempty(obj.agent_battery_texts(i))
                        try
                            delete(obj.agent_battery_texts(i));
                            delete(obj.agent_energy_texts(i));
                            delete(obj.agent_index_texts(i));
                        catch
                            % Text object might already be deleted
                        end
                    end
                    obj.agent_battery_texts(i) = text(-4.5, -3.0, ...
                        sprintf('Agent %d: %.2f%%', i, battery_percentage), ...
                        'FontSize', 8, 'Color', 'black', 'FontWeight', 'bold');
                    obj.agent_energy_texts(i) = text(-4.5, -4.0, ...
                        sprintf('Agent %d: Energy consumed = %.1f', i, e_total), ...
                        'FontSize', 8, 'Color', 'black', 'FontWeight', 'bold');
                    obj.agent_index_texts(i) = text(pos(1)-0.12, pos(2)+0.1, sprintf('%d', agent.index), ...
                     'FontSize', 10, 'FontWeight', 'bold');

                    
                    
                end
            end
            
            % Update time display (top left)
            if isempty(obj.time_text)
                obj.time_text = text(0.02, 0.95, '', 'Units', 'normalized', ...
                                   'FontSize', 12, 'FontWeight', 'bold', ...
                                   'BackgroundColor', 'white');
            end
            set(obj.time_text, 'String', sprintf('Time: %.1f', current_time));
            
            % Update uncertainty display (below time, with better spacing)
            if isempty(obj.uncertainty_text)
                obj.uncertainty_text = text(0.02, 0.85, '', 'Units', 'normalized', ...
                                          'FontSize', 9, 'BackgroundColor', 'white', ...
                                          'VerticalAlignment', 'top');
            end
            % Update total global objective display (below time, with better spacing)
            if isempty(obj.target_global_objective_texts)
                obj.target_global_objective_texts = text(0.02, 0.70, '', 'Units', 'normalized', ...
                                          'FontSize', 9, 'BackgroundColor', 'white', ...
                                          'VerticalAlignment', 'top');
            end
            
            % Create uncertainty summary with better formatting
            uncertainty_str = 'Uncertainty:';
            for i = 1:length(targets)
                uncertainty_str = [uncertainty_str, sprintf('\nT%d: %.1f', ...
                                  targets(i).index, targets(i).uncertainty)];
            end
            set(obj.uncertainty_text, 'String', uncertainty_str);

            
            global_objective = 0;
            for i = 1:length(targets)
                global_objective = global_objective + targets(i).global_objective;
            end
            set(obj.target_global_objective_texts, 'String', sprintf('Global Objective: %.1f', global_objective));


            % Refresh display
            drawnow;
        end

        % plot_energy_battery Visualize agent energy and battery histories
        %   viz.plot_energy_battery(agents)
        %
        % Inputs:
        %   agents - array of Agent objects whose histories were recorded
        %
        % Opens a figure with two subplots:
        %   (1) cumulative energy consumption
        %   (2) battery percentage over time
        function plot_energy_battery(obj, agents)
            fig = figure('Name', 'Agent Energy & Battery History', ...
                         'NumberTitle', 'off', 'Position', [200, 200, 900, 400]);
            tiledlayout(fig, 1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

            % Left subplot: energy
            nexttile;
            hold on; grid on;
            title('Cumulative Energy'); xlabel('Step'); ylabel('Energy (Wh)');
            for i = 1:numel(agents)
                energy = agents(i).e_total_history;
                if isempty(energy); continue; end
                plot(0:numel(energy)-1, energy, 'DisplayName', sprintf('Agent %d', agents(i).index));
            end
            legend('show', 'Location', 'best');

            % middle subplot: battery percentage
            nexttile;
            hold on; grid on;
            title('Battery Percentage'); xlabel('Step'); ylabel('Battery (%)');
            for i = 1:numel(agents)
                soc = agents(i).battery_percentage_history;
                if isempty(soc); continue; end
                plot(0:numel(soc)-1, soc, 'DisplayName', sprintf('Agent %d', agents(i).index));
            end
            legend('show', 'Location', 'best');

            % Right subplot: battery percentage
            nexttile;
            hold on; grid on;
            title('Voltage'); xlabel('Step'); ylabel('Voltage (V)');
            for i = 1:numel(agents)
                volt = agents(i).voltage_history;
                if isempty(volt); continue; end
                plot(0:numel(volt)-1, volt, 'DisplayName', sprintf('Agent %d', agents(i).index));
            end
            legend('show', 'Location', 'best');
        end
        


        % plot_targets_uncertainty Visualize target uncertainty dynamic histories
        %   viz.plot_targets_uncertainty(targets)
        %
        % Inputs:
        %   targets - array of Agent objects whose histories were recorded
        %
        % Opens a figure with two subplots:
        %   (1) target uncertainty
        function plot_targets_uncertainty(obj, targets)
            figure('Name', 'Overall uncertainty', ...
                         'NumberTitle', 'off', 'Position', [200, 200, 900, 400]);
            

            
            hold on; grid on;
            title('Dynamic uncertainty'); xlabel('Step'); ylabel('Uncertainty');
            for i = 1:numel(targets)
                uncertainty = targets(i).history_uncertainty;
                if isempty(uncertainty); continue; end
                plot(0:numel(uncertainty)-1, uncertainty, 'DisplayName', sprintf('Target %d', targets(i).index));
            end
            legend('show', 'Location', 'best');
        end

        % plot_targets_uncertainty Visualize target uncertainty dynamic histories
        %   viz.plot_targets_uncertainty(targets)
        %
        % Inputs:
        %   targets - array of Agent objects whose histories were recorded
        %
        % Opens a figure with two subplots:
        %   (1) target uncertainty
        function plot_overall_objective(obj, targets)
            figure('Name', 'Overall uncertainty', ...
                         'NumberTitle', 'off', 'Position', [200, 200, 900, 400]);
            

            
            hold on; grid on;
            title('Dynamic uncertainty'); xlabel('Step'); ylabel('Uncertainty');
            for i = 1:numel(targets)
                glob_object = targets(i).history_global_objective;
                if isempty(glob_object); continue; end
                plot(0:numel(glob_object)-1, glob_object, 'DisplayName', sprintf('Target %d', targets(i).index));
            end
            legend('show', 'Location', 'best');
        end

        
        function plot_lookup(obj, agents)
            figure('Name', 'Battery Polynomial Lookup Table', ...
                         'NumberTitle', 'off', 'Position', [200, 200, 1000, 600]);
            
            hold on; grid on;
            title('Battery Voltage → SOC Percentage (Polynomial Model)', 'FontSize', 14, 'FontWeight', 'bold');
            xlabel('Voltage (V)', 'FontSize', 12);
            ylabel('Battery Percentage (%)', 'FontSize', 12);
            
            % Plot lookup table for each agent
            for i = 1:numel(agents)
                agent = agents(i);
                
                % Ensure lookup table is built (will build if empty)
                if isempty(agent.lookup_interpolant)
                    agent.lookup(Agent.BAT_MON_V); % Trigger table build
                end
                
                % Plot voltage vs percentage
                if ~isempty(agent.lookup_voltage_table) && ~isempty(agent.lookup_percentage_table)
                    plot(agent.lookup_voltage_table, agent.lookup_percentage_table, ...
                        'LineWidth', 2, 'DisplayName', sprintf('Agent %d', agent.index));
                end
            end
            
            % Add reference lines for min/max voltages
            ylim([0, 100]);
            xlim([Agent.BAT_MIN_V - 0.5, Agent.BAT_MAX_V + 0.5]);
            
            % Vertical lines for voltage bounds
            xline(Agent.BAT_MIN_V, 'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Min Voltage (%.1fV)', Agent.BAT_MIN_V));
            xline(Agent.BAT_MAX_V, 'g--', 'LineWidth', 1.5, 'DisplayName', sprintf('Max Voltage (%.1fV)', Agent.BAT_MAX_V));
            
            legend('show', 'Location', 'best');
        end
    end
end
