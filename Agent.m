classdef Agent < handle
    % AGENT Mobile agent for the persistent monitoring simulation
    %
    % Syntax:
    %   agent = Agent(index, position)
    %
    % Description:
    %   Creates an Agent object that models an autonomous mobile node used to
    %   monitor targets. The agent supports motion control, energy accounting,
    %   and simple decision-making modes (traveling, dwelling, planning,
    %   power outage).
    %
    % Inputs:
    %   index    - integer identifier for the agent
    %   position - 1x2 numeric vector specifying initial [x y] position
    %
    % Key properties:
    %   - position, velocity, mode, battery_percentage, e_total
    %   - energy and coulomb-counting history arrays for analysis
    %
    % Example:
    %   a = Agent(1, [0,0]);
    %   a.set_goal_target([5,5], 3);
    %   a.update_state(0.1);
    %
    % See also: Target, SimulationVisualizer
    properties
        % ===== AGENT STATE =====
        % Agents behavior properties    
        index
        % position: [x, y, theta] TODO: add theta State   
        position

        mode
        mode_history
        % kinematics models
        kinematicsModels

        track_width = 0.3;    % meter
        wheel_radius = 0.05;  % meter

        % controller
        controller

        % linear velocity u
        lin_velocity
        lin_acceleration
        lin_velocity_history
        % angular velocity omega
        ang_velocity
        ang_acceleration
        ang_velocity_history

        % trajectory
        trajectory_turn
        trajectory_move

        % velocity for simulation display
        velocity
        velocity_history

        goal_target
        goal_target_history

        rho
        rho_history

        tau
        tau_history

        planning_time
        planning_time_history

        dwelling_time_remaining
        planning_time_remaining
        available_targets
        current_time   
        
        % ===== ENERGY PROPERTIES =====
        e_total

        motion_power % depend on linear velocity

        % Energy history
        e_cpu_history
        e_acc_history
        e_mot_history
        e_total_history

        soc
        soc_history

        battery_percentage
        battery_percentage_history
        voltage_history

        % coulomb counting history
        q_cpu_history
        q_acc_history
        q_mot_history
        q_total_history

        % OU noise tracking
        % ou_noise
        ou_noise_history
        % ou noise parameters move to constant??
        theta_ou = 0.1;
        mu_ou = 0;
        sigma_ou = 0.2;

        sigma_v = 0.003;

        % Lookup table properties (built once, reused)
        lookup_voltage_table
        lookup_percentage_table
        lookup_interpolant

    end
    properties (Constant)
        % ===== POWER CONSUMPTION (Watts) =====
        % CPU Power - based on official TurtleBot 4 specs
        CPU_PLAN = 6.0;    % Path planning, SLAM (higher computational load)
        CPU_MOVE = 5.5;    % Navigation, obstacle avoidance
        CPU_IDLE = 4.0;    % Standby operations (from Table 1: Raspberry Pi 4B = 4W)

        % ===== INDIVIDUAL COMPONENT POWER (from official TurtleBot 4 power budget) =====
        % Core components - Nominal and Maximum values
        OAK_D_NOMINAL = 5.0;       % OAK-D-Pro camera (nominal)
        OAK_D_MAX = 7.5;           % OAK-D-Pro camera (maximum - for additional AI processing)

        LIDAR_NOMINAL = 2.3;       % RPLIDAR A1M8 (nominal)
        LIDAR_MAX = 3.0;           % RPLIDAR A1M8 (maximum)

        FAN_NOMINAL = 1.3;         % Fan (nominal - for IDLE state)
        FAN_MAX = 2.0;             % Fan (maximum - for active states when robot is working harder)

        UI_BOARD_NOMINAL = 2.4;    % User Interface Board (nominal - includes display, LEDs, USB hub)
        UI_BOARD_MAX = 40.0;       % User Interface Board (maximum - all USB ports at full power)

        % ===== COMPONENT POWER CALCULATION =====
        % Active state: Use maximum fan power when robot is working harder (PLAN, MOVE)
        COMP_ACTIVE = Agent.OAK_D_NOMINAL + Agent.LIDAR_NOMINAL + Agent.FAN_MAX + Agent.UI_BOARD_NOMINAL;
        % COMP_ACTIVE = 5.0 + 2.3 + 2.0 + 2.4 = 11.7W

        % Standby state: Use nominal fan power for IDLE state
        COMP_STANDBY = Agent.OAK_D_NOMINAL + Agent.LIDAR_NOMINAL + Agent.FAN_NOMINAL + Agent.UI_BOARD_NOMINAL;
        % COMP_STANDBY = 5.0 + 2.3 + 1.3 + 2.4 = 11.0W

        % ===== BATTERY SPECIFICATIONS =====
        BAT_MON_V      = 14.4;  % V (nominal voltage)
        BAT_MAX_V      = 16.8;  % V (maximum voltage)
        BAT_MIN_V      = 12.0;  % V (minimal voltage) 0% soc
        BAT_CUTOFF_V   = 10.8;  % V (BMS cutoff)
        BATTERY_E_Wh   = 26.0;  % Wh (capacity)
        R_INTERNAL     = 0.12;  % Ohm (battery internal resistence)

        % ===== MOTION POWER =====
        % Keep Create3 team's measurements as they are based on actual hardware
        % fixed velocity
        % include velocity variation(v , u)
        DRIVE_CURRENT  = 0.526;  % A (0.31m/s linear velocity)
        IDLE_CURRENT   = 0.404;  % A
        MOT_MOVE = Agent.BAT_MON_V * Agent.DRIVE_CURRENT;   % 7.57 W
        MOT_IDLE = Agent.BAT_MON_V * Agent.IDLE_CURRENT;    % 5.82 W

        % ===== EFFICIENCY =====
        ETA_CONV = 0.90;  % DC-DC & wiring loss

        % ===== BATTERY PERCENTAGE CONVERSION =====
        SOC_TO_PERCENTAGE = 10000;      % Multiply SOC by 10000 (0.01% resolution)
        PERCENTAGE_TO_SOC = 0.0001;     % Divide percentage by 10000
        SOC_LOOKUP_POINTS = 10001;

        % ===== POLYNOMIAL COEFFICIENTS =====
        % 17th-order polynomial for LMO discharge (Somakettarin & Funaki 2017)
        k_dis = [3.0016, 2.0082e-1, -5.0440e-2, 1.1287e-2, -1.7962e-3, ...
                1.9382e-4, -1.4444e-5, 7.6498e-7, -2.9500e-8, ...
                8.4261e-10, -1.8003e-11, 2.8847e-13, -3.4474e-15, ...
                3.0256e-17, -1.8926e-19, 7.9827e-22, -2.0344e-24, 2.3659e-27];
        
        % SOC look-up table / information
        OCV_EMPTY = polyval(flip(Agent.k_dis), 0 * 20.0); % 0% SOC - polynomial output at empty
        OCV_FULL  = polyval(flip(Agent.k_dis), 1 * 20.0); % 100% SOC - polynomial output at full
        SCALE_FACTOR = (Agent.BAT_MAX_V - Agent.BAT_MIN_V) / (Agent.OCV_FULL - Agent.OCV_EMPTY);

    end    
    methods
        % Agent constructor
        %   obj = Agent(index, position)
        %
        % Inputs:
        %   index    - integer identifier for the agent
        %   position - 1x3 numeric vector specifying initial [x y theta] position
        function obj = Agent(index, position)
            obj.index = index;
            obj.kinematicsModels = differentialDriveKinematics('TrackWidth', obj.track_width,...
                'WheelRadius', obj.wheel_radius);
            obj.controller = controllerPurePursuit;
            obj.position = position;    
            obj.goal_target = [0 0 0]; % default goal target is [0,0,0]
            obj.mode = "idle"; % default mode is idle
            obj.velocity = [0, 0];
            obj.dwelling_time_remaining = 0;
            obj.current_time = 0;
            obj.planning_time_remaining = 0;
            obj.mode_history = [];
            obj.velocity_history = [];
            obj.goal_target_history = [];
            obj.rho_history = [];
            obj.tau_history = [];
            obj.planning_time_history = [];
            
             % Initialize energy properties
            obj.e_total = 0;
             % OU noise history

            obj.ou_noise_history = [0];

            obj.e_cpu_history = [];
            obj.e_acc_history = [];
            obj.e_mot_history = [];
            obj.e_total_history = [0]; % start with zero energy consumed

            obj.q_cpu_history = [];
            obj.q_acc_history = [];
            obj.q_mot_history = [];
            obj.q_total_history = [];

            obj.battery_percentage = 100;
            obj.battery_percentage_history = [100];

            obj.soc = 1;
            obj.soc_history = [1];
            obj.voltage_history = [];
        end

        % set_goal_target Assign a new target and travel time (rho)
        %   obj = obj.set_goal_target(goal_target, rho)
        %
        % Inputs:
        %   goal_target - 1x2 numeric vector of target [x y]
        %   rho         - desired travel time to reach the goal (seconds)
        % TODO modify this and add trajectory calculation waypointTrajectory or
        % polynomialTrajectory  They have parameter of arrival time ; Sync
        % the frequency, then update accordingly
        % 
        function obj = set_goal_target(obj, goal_target, rho, delta_time)

            obj.mode = "traveling"; % set the mode to traveling
            obj.mode_history = [obj.mode_history, "traveling"];
            
            % TODO: goal target theta value should be the direction that current position to goal target
            goal_target_direction = atan2(goal_target(2) - obj.position(2), goal_target(1) - obj.position(1));
            obj.goal_target = [goal_target(1), goal_target(2), goal_target_direction];
            obj.goal_target_history = [obj.goal_target_history; goal_target];

            % TODO: add a middle waypoint to the trajectory as the turnning point 
            middle_waypoint = [obj.position(1), obj.position(2), goal_target_direction];
            
            obj.rho = rho;
            obj.rho_history = [obj.rho_history; rho];
            waypoints_turn = [obj.position;middle_waypoint];
            waypoints_move = [middle_waypoint;obj.goal_target];
            % TODO: add a time of arrival for the middle waypoint turning should be a small portion of the total travel time
            
            obj.trajectory_turn = waypointTrajectory(Waypoints=waypoints_turn,...
                SampleRate=1/delta_time,TimeOfArrival=[0, rho/5]);
            obj.trajectory_move = waypointTrajectory(Waypoints=waypoints_move,...
                SampleRate=1/delta_time,TimeOfArrival=[rho/5, rho] );
            
            

            obj.current_time = 0; % Initialize current time counter
            
        end

        % set_dwelling_time Put the agent into dwelling mode for tau seconds
        %   obj = obj.set_dwelling_time(tau)
        %
        % Inputs:
        %   tau - dwelling time (seconds)
        function obj = set_dwelling_time(obj, tau)
                obj.mode = "dwelling"; % set the mode to dwelling
                obj.mode_history = [obj.mode_history, "dwelling"];

                obj.tau = tau;
                obj.tau_history = [obj.tau_history, tau];

                obj.dwelling_time_remaining = tau;
                
            end

        % set_available_targets Store a list of target objects the agent can visit
        %   obj = obj.set_available_targets(targets)
        %
        % Inputs:
        %   targets - array of Target objects
        function obj = set_available_targets(obj, targets)
            obj.available_targets = targets;
        end

        % find current index 
        function current_target_idx = current_index(obj)
            current_target_idx = 0;
            for i = 1:length(obj.available_targets)
                if norm(obj.position(1:2) - obj.available_targets(i).position) < 0.2
                    current_target_idx = i;
                    break;
                end
            end
        end


        % motion_energy_calculation Calculate motion power with velocity dependence
        %   obj = obj.motion_energy_calculation()
        %
        % Inputs:
        %   none
        %
        % Outputs:
        %   obj - updated agent object
        %
        % This method calculates the motion power based on the linear velocity.
        function obj = motion_energy_calculation(obj)
            % Calculate motion power with velocity dependence
            drive_current = 0.404 + 0.122/ 0.31 * obj.lin_velocity;  % A (linear velocity dependent) 
            obj.motion_power = 14.4 * drive_current; % W

            
        end
        % energy_calculation Compute energy usage and update battery state
        %   obj.energy_calculation(delta_time)
        %
        % Inputs:
        %   delta_time - simulation time step (seconds)
        %
        % This method updates instantaneous power, accumulates energy usage,
        % performs coulomb-counting, updates SOC and estimates terminal voltage.
        % It also records both coulomb-counting and voltage-based SOC estimates.
        function energy_calculation(obj, delta_time)
            % Calculate OU noise
            ou_noise = obj.ou_noise_history(end) + obj.theta_ou * (obj.mu_ou - obj.ou_noise_history(end)) * delta_time + obj.sigma_ou * sqrt(delta_time) * randn();
            obj.ou_noise_history = [obj.ou_noise_history, ou_noise];
            % Calculate insntantaneous power with OU noise (+- 20% variation)
            % refresh motion energy calculation
            obj = motion_energy_calculation(obj);

            switch obj.mode
                case "planning"
                    p_cpu = obj.CPU_PLAN * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_ACTIVE * (1 + 0.2 * ou_noise);
                    p_mot = obj.motion_power * (1 + 0.2 * ou_noise);
                case "traveling"
                    p_cpu = obj.CPU_MOVE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_ACTIVE * (1 + 0.2 * ou_noise);
                    p_mot = obj.motion_power * (1 + 0.2 * ou_noise);
                case "dwelling"
                    p_cpu = obj.CPU_IDLE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_STANDBY * (1 + 0.2 * ou_noise);
                    p_mot = obj.motion_power * (1 + 0.2 * ou_noise);
                case "idle"
                    p_cpu = obj.CPU_IDLE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_STANDBY * (1 + 0.2 * ou_noise);
                    p_mot = obj.motion_power * (1 + 0.2 * ou_noise);
                otherwise
                    p_cpu = obj.CPU_IDLE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_STANDBY * (1 + 0.2 * ou_noise);
                    p_mot = obj.motion_power * (1 + 0.2 * ou_noise);
            end
            % Apply conversion efficiency loss total power demand
            p_total = (p_cpu + p_acc + p_mot) / Agent.ETA_CONV;

            % Accumulate enery over time
            e_cpu = p_cpu * delta_time;
            e_acc = p_acc * delta_time;
            e_mot = p_mot * delta_time;
            obj.e_cpu_history = [obj.e_cpu_history, e_cpu];
            obj.e_acc_history = [obj.e_acc_history, e_acc];
            obj.e_mot_history = [obj.e_mot_history, e_mot];

            % Total enrgy (with efficiency loss)
            obj.e_total = (p_total * delta_time) + obj.e_total_history(end);

            obj.e_total_history = [obj.e_total_history,obj.e_total];


            
            %% Obtain true SOC through coulomb counting  
            % Update battery state
            soc_current = obj.soc_history(end);
            ocv_polynomial = polyval(flip(Agent.k_dis), soc_current * 20.0);
            ocv_current = Agent.BAT_MIN_V + (ocv_polynomial - Agent.OCV_EMPTY) * Agent.SCALE_FACTOR;

            i_current = (ocv_current - sqrt(ocv_current^2 - 4 * Agent.R_INTERNAL * p_total)) /2 * Agent.R_INTERNAL;
            % Safety check for negative current
            if i_current < 0
                i_current = 0;
            end
            
            % Q = E / V = (Battery energy Wh * 3600 s/h) / nominal voltage V
            Q_battery_C = (Agent.BATTERY_E_Wh * 3600) / Agent.BAT_MON_V;
            soc_new = soc_current - (i_current * delta_time) / Q_battery_C;
            obj.soc_history = [obj.soc_history, soc_new];
            obj.battery_percentage = soc_new * Agent.SOC_TO_PERCENTAGE / 100;
            obj.battery_percentage_history = [obj.battery_percentage_history, obj.battery_percentage];


            % Calculate v_terminal 
            % R_0 internal resistent dynamic flat at the middle spike at the soc high and low *(1+0.2*(1-4*soc_new*(1-soc_new))+0.4*((1-soc_new)^2))
            v_terminal = ocv_current - Agent.R_INTERNAL*i_current;
            obj.voltage_history = [obj.voltage_history, v_terminal];
            
        end

        

    % update_state Advance agent state by delta_time
    %   obj = obj.update_state(delta_time)
    %
    % Inputs:
    %   delta_time - simulation time step (seconds)
    %
    % The method updates position, manages mode transitions, and calls
    % energy_calculation to update battery state.
    function obj = update_state(obj, delta_time)
            switch obj.mode
                case "power_outage"
                    % Robot stops moving when battery is below cutoff voltage
                    obj.velocity = [0, 0];
                    

                case "traveling"
                    % TODO not just update trajectory
                    % update the agent position
                    % turning trajectory and moving trajectory update
                    if obj.current_time <= obj.rho/5
                        trajectory = obj.trajectory_turn;
                    else
                        trajectory = obj.trajectory_move;
                    end
                    [position_traj, ~, velocity_traj, acceleration_traj, ~] = lookupPose(trajectory, obj.current_time);
                    obj.position = position_traj';
                    velocity_traj = velocity_traj';
                    obj.velocity = velocity_traj(1:2);
                    obj.lin_velocity = sqrt(velocity_traj(1)^2 + velocity_traj(2)^2);
                    obj.lin_acceleration = sqrt(acceleration_traj(1)^2 + acceleration_traj(2)^2);
                    obj.ang_velocity = velocity_traj(3);
                    obj.ang_acceleration = acceleration_traj(3);
                    % obj.lin_velocity_history = [obj.lin_velocity_history, obj.lin_velocity];
                    % obj.lin_acceleration_history = [obj.lin_acceleration_history, obj.lin_acceleration];
                    % obj.ang_velocity_history = [obj.ang_velocity_history, obj.ang_velocity];
                    % obj.ang_acceleration_history = [obj.ang_acceleration_history, obj.ang_acceleration];

                    obj.current_time = obj.current_time + delta_time;
                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);
                    if obj.battery_percentage <= 10
                        obj.mode = "power_outage";
                        return; % is this needed? what can it do ?
                    end
                    % check trajectory content, dump out every detail then
                    % swith mode. (abandoned time and distance checked)
                    % Count down travel time
                    distance_to_goal = norm(obj.position(1:2) - obj.goal_target(1:2));
                    if distance_to_goal <= 0.05 || obj.current_time >= obj.rho
                        obj.position = obj.goal_target;
                        obj.mode = "planning";
                        obj.planning_time = rand(); % random planning time between 0-1s
                        obj.planning_time_remaining = obj.planning_time;
                    end
                    
                    
                    
                case "dwelling"
                    % stay at target position
                    
                    obj.velocity = [0, 0];

                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);
                    if obj.battery_percentage <= 10
                        obj.mode = "power_outage";
                        return; % is this needed? what can it do ?
                    end

                    % count down dwelling time
                    obj.dwelling_time_remaining = obj.dwelling_time_remaining - delta_time;
                    
                    % Check if dwelling time is finished (trigger to new random target)
                    if obj.dwelling_time_remaining <= 0
                        % Find which target we're currently at
                        obj.mode = "planning";
                        obj.planning_time = rand(); % random planning time between 0-1s
                        obj.planning_time_remaining = obj.planning_time;
                    end

                case "idle" % defalut mode
                    % do nothing, waiting for new target assignment
                    obj.velocity = [0, 0];
                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);


                case "planning"
                    % embedded the decision making process here
                    obj.velocity = [0, 0];
                    obj.planning_time_remaining = obj.planning_time_remaining - delta_time;

                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);
                    if obj.battery_percentage <= 10
                        obj.mode = "power_outage";
                        return; 
                    end

                    % Check if planning time is finished
                    if obj.planning_time_remaining <= 0
                    % while(obj.planning_time_remaining <= 0)
                        if obj.mode_history(end) == "dwelling"
                        
                            obj.mode_history = [obj.mode_history, "planning"];
                            % ===== Random destination selection =====
                            % Select a different target
                            % Find which target agent currently at
                            current_target_idx = obj.current_index();
                            
                            % make a available targets list
                            if current_target_idx > 0
                                avalible_indices = setdiff(1:length(obj.available_targets), current_target_idx);
                            else 
                                avalible_indices = 1:length(obj.available_targets);
                            end

                            % Grabbing essential data to process Reciding Horizon 
                            r0i = [obj.available_targets.uncertainty]';          % Uncertaitny value from all target at that current time                      
                            Ai = [obj.available_targets.A]';                     % Uncertainty grwoing parameter value from all target  
                            Bi = [obj.available_targets.B]';                     % Ucertainty decreasing paratmeter value from all target
                            all_target_indices = [obj.available_targets.index]'; % All target index

                            % variable to find mininmal objective value
                            J_temp = inf;
                            for i = 1:length(avalible_indices)
                                % try all possible goal target 
                                goal_target_idx = avalible_indices(i);
                                x0 = [1; 1];
                                lb = [1; 1];
                                ub = [inf; inf];
                                % find the optimal decision through fmincon, (objective travel gives the objective value of 3 events horizon) See Function below
                                [x_opt,J_opt] = fmincon(@(x) Agent.objective_travel(x(1), x(2),  ...
                                    r0i, Ai, Bi,  all_target_indices, goal_target_idx), ...
                                    x0,[],[],[],[],lb, ub);
                                if J_temp > J_opt
                                    J_temp = J_opt;
                                    % Only execute the first event with target id and correspond rho value
                                    target_idx = goal_target_idx;
                                    new_rho = x_opt(1);
                                end
                            end
                            obj.set_goal_target([obj.available_targets(target_idx).position,0], new_rho, delta_time)

                            
                           
                            
                        elseif obj.mode_history(end) == "traveling"
                            obj.mode_history = [obj.mode_history, "planning"];
                            %========= Finding optimze dwelling time using event driven receding horizon=======%
                            % locate current target index
                            current_target_idx = obj.current_index();
                            
                            % find out the avalible targets index to visit
                            if current_target_idx > 0
                                avalible_indices = setdiff(1:length(obj.available_targets), current_target_idx);
                            else 
                                avalible_indices = 1:length(obj.available_targets);
                            end
                            % Grabbing essential data to process Reciding Horizon 
                            r0i = [obj.available_targets.uncertainty]';          % Uncertaitny value from all target at that current time                      
                            Ai = [obj.available_targets.A]';                     % Uncertainty grwoing parameter value from all target  
                            Bi = [obj.available_targets.B]';                     % Ucertainty decreasing paratmeter value from all target
                            all_target_indices = [obj.available_targets.index]'; % All target index

                            % variable to find mininmal objective value
                            J_temp = inf;
                            for i = 1:length(avalible_indices)
                                goal_target_idx = avalible_indices(i);
                                % try all posible target
                                x0 = [1; 1; 1];
                                lb = [0; 0; 0];
                                ub = [inf; inf; inf];
                                % find the optimal decision through fmincon (objective dwell gives the objective value of 2 events horizon) See Function below)
                                [x_opt,J_opt] = fmincon(@(x) Agent.objective_dwell(x(1), x(2), x(3), ...
                                    r0i, Ai, Bi, ...
                                    current_target_idx, all_target_indices, goal_target_idx), ...
                                    x0, [], [], [], [], lb, ub);
                                if J_temp > J_opt
                                    J_temp = J_opt;
                                    % Only execute the first event with dwelling time tau
                                    new_tau = x_opt(1);
                                end
                            end
                            obj.set_dwelling_time(new_tau);
                        end
                    
                    end


                    
            end       
        end
        % SOC lookup table (voltage based)
        function battery_percentage_volt = lookup(obj, voltage)
            % Build the lookup table only once and reuse it on subsequent calls
            
            if isempty(obj.lookup_interpolant)
                % Build lookup table once
                soc_samples = linspace(0, 1, Agent.SOC_LOOKUP_POINTS);
                ocv_values = polyval(flip(Agent.k_dis), soc_samples * 20.0); % Evaluate OCV curve
                voltage_table = Agent.BAT_MIN_V + (ocv_values - Agent.OCV_EMPTY) * Agent.SCALE_FACTOR;
                percentage_table = soc_samples * Agent.SOC_TO_PERCENTAGE / 100; % Convert SOC to percent
                
                % Sort by voltage for interpolation
                [voltage_table, sort_idx] = sort(voltage_table);
                percentage_table = percentage_table(sort_idx);
                
                % Store tables and create interpolant
                obj.lookup_voltage_table = voltage_table;
                obj.lookup_percentage_table = percentage_table;
                obj.lookup_interpolant = griddedInterpolant(voltage_table, percentage_table, 'pchip', 'nearest');
                
                % Debug: Verify lookup table range
                fprintf('Lookup table built: min_voltage=%.4fV (%.1f%%), max_voltage=%.4fV (%.1f%%), expected_max=%.4fV\n', ...
                    voltage_table(1), percentage_table(1), voltage_table(end), percentage_table(end), Agent.BAT_MAX_V);
            end

            % Use the stored interpolant
            voltage_clamped = min(max(voltage, Agent.BAT_MIN_V), Agent.BAT_MAX_V);
            battery_percentage_volt = obj.lookup_interpolant(voltage_clamped);
            battery_percentage_volt = max(0, min(100, battery_percentage_volt));
            
            % Debug: Check lookup result
            if voltage > 16.0  % Only debug high voltages
                fprintf('DEBUG lookup: input=%.4fV, clamped=%.4fV, result=%.2f%%\n', ...
                    voltage, voltage_clamped, battery_percentage_volt);
            end
        end    

    end
    methods (Static) 
        
        % RHCP method placeholder
        % this is where agent should be find the optimal decision based on RHCP

        % uncertainty_mon Estimate uncertainty after monitored interval
        %   r = Agent.uncertainty_mon(r0, A, B, t)
        %
        % Inputs:
        %   r0 - current uncertainty
        %   A  - natural growth rate
        %   B  - reduction rate while observed
        %   t  - monitored duration
        %
        % Output:
        %   r  - uncertainty after time t, clamped at zero if saturated
        function r = uncertainty_mon(r0, A, B ,t)
            t_sat = -r0 / (A-B);
            if t_sat >= t
                r = r0 + (A -B)*t;
            else
                r = r0 + (A - B)*t_sat; % r = 0
            end
        end
        
        % uncertainty_unmon Propagate uncertainty while unmonitored
        %   r = Agent.uncertainty_unmon(r0, A, t)
        %
        % Inputs:
        %   r0 - current uncertainty
        %   A  - growth rate
        %   t  - unmonitored duration
        %
        % Output:
        %   r  - increased uncertainty after time t
        function r = uncertainty_unmon(r0, A, t)
            r = r0 + A*t;
        end
              
        % Dwell Evaluate dwell-phase objective contribution
        %   J = Agent.Dwell(r0_sum, A_sum, r0, A, B, tau)
        %
        % Inputs:
        %   r0_sum - total initial uncertainty across all targets
        %   A_sum - total growth rate across all targets
        %   r0    - initial uncertainty of the serviced target
        %   A     - growth rate of the serviced target
        %   B     - reduction rate of the serviced target
        %   tau   - dwell duration
        %
        % Output:
        %   J     - dwell cost contribution for horizon averaging
        function J = Dwell(r0_sum, A_sum,r0, A, B, tau)
            t_sat = -r0 / (A-B);
            if t_sat >= tau
                J = r0_sum + 0.5*(A_sum - B)*tau^2;
            else
                J = (r0_sum - r0 + 0.5*(A_sum - A)*tau^2) + 0.5 * r0 * t_sat;
            end 
            
        end
              
        % Travel Evaluate travel-phase objective contribution
        %   J = Agent.Travel(r0_sum, A_sum, rho)
        %
        % Inputs:
        %   r0_sum - total uncertainty baseline
        %   A_sum - total growth rate across targets
        %   rho   - travel duration
        %
        % Output:
        %   J     - travel cost contribution for horizon averaging
        function J = Travel(r0_sum,A_sum, rho)
            J = r0_sum + 0.5*A_sum*rho^2;
        end

        % objective_dwell Horizon cost for dwell-travel-dwell schedule
        %   J = Agent.objective_dwell(tau1, rho1, tau2, r0i, Ai, Bi, current_idx, all_idx, goal_idx)
        %
        % Inputs:
        %   tau1        - first dwell duration at current target
        %   rho1        - travel duration to candidate goal
        %   tau2        - dwell duration at candidate goal
        %   r0i, Ai, Bi - vectors of uncertainties and rates for all targets
        %   current_idx - index of currently serviced target
        %   all_idx     - list of target indices
        %   goal_idx    - index of candidate goal target
        %
        % Output:
        %   J           - average cost over the combined horizon
        function J = objective_dwell(tau1, rho1 , tau2, r0i, Ai, Bi, current_target_idx, all_target_indices, goal_target_idx)
            % consist with dwell->travel->dwell

            % r0i and Ai and Bi are matrix that store all targets correspond value
            r0_sum = sum(r0i);
            A_sum = sum(Ai);

            %% first dwell time
            % find the index of target that agent is monitored
            J_first_dwell = Agent.Dwell(r0_sum, A_sum,r0i(current_target_idx),Ai(current_target_idx), Bi(current_target_idx), tau1);

            % update r0_sum for next event // current target being monitored
            for i = 1: length(all_target_indices)
                if i == current_target_idx
                    r0i(current_target_idx) = Agent.uncertainty_mon(r0i(current_target_idx), Ai(current_target_idx),Bi(current_target_idx), rho1);
                else
                    r0i(i) = Agent.uncertainty_unmon(r0i(i), Ai(i), tau1);
                end
            end
            r0_sum = sum(r0i);

            %% first travel time
            J_first_travel = Agent.Travel(r0_sum, A_sum, rho1);

            % update r0_sum for next event // all targets are not being monitored
            for i = 1: length(all_target_indices)
                r0i(i) = Agent.uncertainty_unmon(r0i(i), Ai(i), tau1);
            end
            %% Second dwell time 
            J_second_dwell = Agent.Dwell(r0_sum, A_sum, r0i(goal_target_idx),Ai(goal_target_idx),Bi(goal_target_idx), tau2);


            % combine 
            J = (J_first_dwell + J_first_travel + J_second_dwell) / (tau1 + rho1 + tau2);

        end

        % objective_travel Horizon cost for travel-dwell schedule
        %   J = Agent.objective_travel(rho1, tau1, r0i, Ai, Bi, all_idx, goal_idx)
        %
        % Inputs:
        %   rho1        - travel duration to candidate goal
        %   tau1        - dwell duration at candidate goal
        %   r0i, Ai, Bi - vectors of uncertainties and rates for all targets
        %   all_idx     - list of target indices
        %   goal_idx    - index of candidate goal target
        %
        % Output:
        %   J           - average cost for the two-phase horizon
        function J = objective_travel(rho1 , tau1, r0i, Ai, Bi, all_target_indices, goal_target_idx)
            % consist with travel->dwell

            % r0i and Ai and Bi are matrix that store all targets correspond value
            r0_sum = sum(r0i);
            A_sum = sum(Ai);

            %% first travel time
            J_first_travel = Agent.Travel(r0_sum, A_sum, rho1);

            % update r0_sum for next event // all targets are not being monitored
            for i = 1: length(all_target_indices)
                r0i(i) = Agent.uncertainty_unmon(r0i(i), Ai(i), rho1);
            end
            %% first dwell time 
            J_first_dwell = Agent.Dwell(r0_sum, A_sum,r0i(goal_target_idx), Ai(goal_target_idx),Bi(goal_target_idx), tau1);

            % combine 
            J = (J_first_travel + J_first_dwell) / (rho1 + tau1 );

        end

    end
end
