classdef Target < handle
    % TARGET Target class for persistent monitoring simulation
    %
    % Syntax:
    %   target = Target(index, position, uncertainty, A, B)
    %
    % Description:
    %   Target models a monitoring location with an uncertainty value that
    %   evolves over time. Uncertainty grows at rate A when unmonitored and
    %   decreases at rate B when an agent is present.
    %
    % Inputs:
    %   index       - integer identifier for the target
    %   position    - 1x2 numeric vector specifying [x y] coordinates
    %   uncertainty - initial uncertainty value (numeric)
    %   A           - uncertainty growth rate when unmonitored
    %   B           - uncertainty reduction rate when monitored
    %
    % Example:
    %   t = Target(1, [0,0], 10, 2.5, 10);
    %   [R, obj, g] = t.update_state(0.1, [0,0]);
    %
    % See also: Agent
    properties
        index
        mode

        position
        uncertainty
        A   % uncertainty rising rate
        B   % uncertaitny decreaseing rate
        objective
        global_objective
        history_uncertainty
        history_objective
        history_global_objective
        history_time
    end
    methods
        % Target constructor
        %   obj = Target(index, position, uncertainty, A, B)
        %
        % Inputs:
        %   index       - integer identifier for the target
        %   position    - 1x2 numeric vector specifying [x y] coordinates
        %   uncertainty - initial uncertainty value
        %   A           - uncertainty growth rate (unmonitored)
        %   B           - uncertainty reduction rate (monitored)
        function obj = Target(index, position, uncertainty, A, B)
            obj.index = index;
            obj.position = position;
            obj.uncertainty = uncertainty;  % initial uncertainty
            obj.A = A;
            obj.B = B;
            obj.objective = 0;
            obj.global_objective = 0;
            obj.mode = "unmonitor";  % default mode
            obj.history_uncertainty = [];
            obj.history_objective = [];
            obj.history_global_objective = [];
            obj.history_time = [];
        end

        % mode_switch Update target mode based on agent proximity
        %   obj = obj.mode_switch(agent_position)
        %
        % Inputs:
        %   agent_position - 1x2 numeric vector of agent [x y]
        %
        % Sets obj.mode to 'monitor' when the agent is at the same position
        % (exact match) and to 'unmonitor' otherwise.
        function obj = mode_switch(obj, agent_position)
            if obj.position == agent_position
                obj.mode = "monitor";
            else
                obj.mode = "unmonitor";
            end
        end

    % update_state Advance target state by delta_time and update objectives
    %   [R, obj_val, gobj] = obj.update_state(delta_time, agent_position)
    %
    % Inputs:
    %   delta_time     - simulation time step (seconds)
    %   agent_position - 1x2 numeric vector of agent [x y]
    %
    % Outputs:
    %   R_value                - updated uncertainty
    %   objective_value        - local objective for this time step
    %   global_objective_value - cumulative/global objective estimate
    function [R_value, objective_value, global_objective_value] = update_state(obj, delta_time, agent_position)
            total_time = 0;
            obj.history_uncertainty = [obj.history_uncertainty, obj.uncertainty];  % update history
            obj.history_objective = [obj.history_objective, obj.objective];  % update history
            obj.history_global_objective = [obj.history_global_objective, obj.global_objective];  % update history
            total_time = total_time + delta_time;
            obj.history_time = [obj.history_time, total_time];  % update history
            obj.mode_switch(agent_position);
            switch obj.mode 
                case "monitor"
                    [obj.uncertainty, obj.objective, obj.global_objective] = obj.compute_state(delta_time);
                
                case "unmonitor"
                    R_initial = obj.uncertainty;
                    obj.uncertainty = R_initial + obj.A * delta_time;

                    obj.objective = (R_initial + obj.uncertainty) * delta_time / 2;
                    obj.global_objective = obj.objective / delta_time;
                case "pause"
                    obj.uncertainty = obj.uncertainty;
            
            end
            
            R_value = obj.uncertainty;
            objective_value = obj.objective;
            global_objective_value = obj.global_objective;
        end
        
    % compute_state Compute uncertainty and objective when monitored
    %   [R, obj_val, gobj] = obj.compute_state(delta_time)
    %
    % Inputs:
    %   delta_time - simulation time step (seconds)
    %
    % Outputs:
    %   R_value                - updated uncertainty (clamped >= 0)
    %   objective_value        - objective accumulated during delta_time
    %   global_objective_value - normalized/global objective value
    %
    % Notes:
    %   Assumes a single monitoring agent (N = 1). If multiple agents are
    %   present, modify N accordingly.
    function [R_value, objective_value, global_objective_value] = compute_state(obj, delta_time)
            % computer uncertaintu value
            R_initial = obj.uncertainty;
            N = 1; % thinking I can set up a check function to check how many agents are monitoring this target
            % Maybe use mode_switch function
            R_unsaturated = R_initial + obj.A * delta_time - obj.B * N * delta_time;
            R_value = max(0, R_unsaturated);

            % copute objective value            # Create an ed25519 key (recommended). Replace email with your GitHub email.
            ssh-keygen -t ed25519 -C "you@example.com"
            # Accept defaults (press Enter) and optionally set a passphrase.            cat ~/.ssh/id_ed25519.pub
            if R_unsaturated <= 0
                saturation_time = R_initial / -(obj.A - obj.B * N);
                objective_value = (saturation_time * R_initial) / 2;
            else
                objective_value = (R_initial + R_unsaturated) * delta_time / 2;
            end

            % compute global objective value for one target as the horizon spec
            global_objective_value = objective_value / obj.history_time(end);
        end

        

        
        
    end
end
                