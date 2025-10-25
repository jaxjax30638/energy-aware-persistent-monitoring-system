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
        uncertainty_text
        time_text
        xlim_range
        ylim_range
        target_uncertainty_texts
        target_objective_texts
        target_global_objective_texts
        agent_battery_texts
        agent_energy_texts
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
                
                % Create agent plot (triangle)
                plot_handle = plot(pos(1), pos(2), '^', 'MarkerSize', 12, ...
                                 'MarkerFaceColor', 'blue', 'MarkerEdgeColor', 'black', 'LineWidth', 2);
                
                % Add agent index text
                text(pos(1)-0.5, pos(2)-0.5, sprintf('A%d', agent.index), ...
                     'FontSize', 10, 'FontWeight', 'bold');
                
                % Store plot handle
                obj.agent_plots = [obj.agent_plots, plot_handle];
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
                    
                    % Update agent position
                    set(obj.agent_plots(i), 'XData', pos(1), 'YData', pos(2));

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

                    % Update agent battery text
                    if length(obj.agent_battery_texts) >= i && ~isempty(obj.agent_battery_texts(i))
                        try
                            delete(obj.agent_battery_texts(i));
                            delete(obj.agent_energy_texts(i));
                        catch
                            % Text object might already be deleted
                        end
                    end
                    obj.agent_battery_texts(i) = text(pos(1)-0.5, pos(2)+0.5, ...
                        sprintf('%.2f%%', battery_percentage), ...
                        'FontSize', 8, 'Color', 'black', 'FontWeight', 'bold');
                    obj.agent_energy_texts(i) = text(pos(1)-0.5, pos(2)+1.0, ...
                        sprintf('E=%.1f', e_total), ...
                        'FontSize', 8, 'Color', 'white', 'FontWeight', 'bold');

                    % Update agent energy text
                    if length(obj.agent_energy_texts) >= i && ~isempty(obj.agent_energy_texts(i))
                        try
                            delete(obj.agent_energy_texts(i));
                        catch
                            % Text object might already be deleted
                        end
                    end
                    obj.agent_energy_texts(i) = text(pos(1)+0.5, pos(2)+1.0, ...
                        sprintf('E=%.1f', agent.e_total), ...
                        'FontSize', 8, 'Color', 'black', 'FontWeight', 'bold');
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
    end
end
