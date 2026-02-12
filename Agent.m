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
    %   position - 1x3 numeric vector specifying initial [x y theta] position
    %
    % Key properties:
    %   - position, mode, battery_percentage, e_total
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
        % position: [x, y, theta]
        position

        mode
        mode_history  % Mode at each time step (matches e_total_history length)

        % linear velocity u
        lin_velocity
        lin_acceleration
        % angular velocity omega
        ang_velocity
        ang_acceleration
        % trajectory
        trajectory

        goal_target

        rho

        tau

        planning_time
        dwelling_time_remaining
        planning_time_remaining
        available_targets
        current_time   
        
        % ===== ENERGY PROPERTIES =====
        e_total

        motion_power % depend on linear velocity

        % Energy history
        e_total_history

        soc
        soc_history

        battery_percentage
        battery_percentage_history
        voltage_history

        % OU noise tracking
        ou_noise_history
        % OU noise parameters
        theta_ou = 0.1;
        mu_ou = 0;
        sigma_ou = 0.2;

        % Lookup table properties (built once, reused)
        lookup_voltage_table
        lookup_percentage_table
        lookup_interpolant

        % Phase data for energy prediction model
        phase_data

        % Receding Horizon Log
        RH_log

        % prediction model
        travel_predict
        dwell_predict
        plan_predict
        K

    end
    properties (Constant)
        % ===== PHYSICAL PROPERTIES =====
        MASS = 10.0;              % kg - robot mass (to be measured/specified)
        ROLLING_FRICTION = 0.02;  % dimensionless - rolling friction coefficient (typical 0.01-0.05)
        GRAVITY = 9.81;           % m/s²
        V_MAX = 5.0;              % m/s - maximum velocity
        U_MAX = 2.0;              % m/s² - maximum acceleration
        ROTATION_SPEED = 10.0;    % rad/s - maximum rotation speed

        % ===== MOTION ENERGY COEFFICIENTS =====
        P0_BASE_MOTION = 5.82;    % W - from MOT_IDLE (measured idle motor power)
        ALPHA_VELOCITY = Agent.ROLLING_FRICTION * Agent.MASS * Agent.GRAVITY;  % W·s/m - rolling resistance
        GAMMA_ACCELERATION = 0.5; % W·s³/m² - acceleration losses (tuned, sensitivity analysis pending) 
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
        % Not using these values anymore, using the energy-optimal velocity profile instead
        % Used motion_energy_calculation() instead
        %DRIVE_CURRENT  = 0.526;  % A (0.31m/s linear velocity)
        %IDLE_CURRENT   = 0.404;  % A
        %MOT_MOVE = Agent.BAT_MON_V * Agent.DRIVE_CURRENT;   % 7.57 W
        %MOT_IDLE = Agent.BAT_MON_V * Agent.IDLE_CURRENT;    % 5.82 W

        % ===== EFFICIENCY =====
        ETA_CONV = 0.90;  % DC-DC & wiring loss

        % ===== BATTERY PERCENTAGE CONVERSION =====
        SOC_TO_PERCENTAGE = 10000;      % Multiply SOC by 10000 (0.01% resolution)
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
            obj.position = position;    
            obj.goal_target = [0 0 0]; % default goal target is [0,0,0]
            obj.mode = "idle"; % default mode is idle
            obj.mode_history = ["idle"]; % Initialize with starting mode
            obj.lin_velocity = 0;
            obj.lin_acceleration = 0;
            obj.ang_velocity = 0;
            obj.ang_acceleration = 0;
            obj.dwelling_time_remaining = 0;
            obj.current_time = 0;
            obj.planning_time_remaining = 0;
            
            % Initialize energy properties
            obj.e_total = 0;
            obj.ou_noise_history = [0];

            obj.e_total_history = [0]; % start with zero energy consumed

            obj.battery_percentage = 100;
            obj.battery_percentage_history = [100];

            obj.soc = 1;
            obj.soc_history = [1];
            obj.voltage_history = [];

            obj.phase_data = struct('phase_type', {}, 'duration', {},'distance', {}, ...
                'energy_start', {}, 'energy_end', {}, 'energy_consumed', {}, ...
            'battery_start', {}, 'battery_end', {}, 'battery_delta', {}, ...
            'rho', {}, 'tau', {}, 'planning_time', {}, ...
            'start_time', {}, 'end_time', {});
            obj.RH_log = struct('optimal', {}, 'current_idx', {}, 'goal_idx', {}, 'J_opt', {}, ...
            'opt_values', {}, 'J_uncertainty', {}, 'E_bar', {}); 

            S = load("energy_model_travel.mat");
            obj.travel_predict = S.energy_model_travel;
            S = load("energy_model_dwell.mat");
            obj.dwell_predict = S.energy_model_dwell;
            S = load("energy_model_plan.mat");
            obj.plan_predict = S.energy_model_plan;
            obj.K = 1.0; % energy awareness coefficient
        end

        % set_goal_target Assign a new target and travel time (rho)
        %   obj = obj.set_goal_target(goal_target, rho)
        %
        % Inputs:
        %   goal_target - 1x3 numeric vector of target [x y theta]
        %   rho         - desired travel time to reach the goal (seconds)
        % TODO modify this and add trajectory calculation waypointTrajectory or
        % polynomialTrajectory  They have parameter of arrival time ; Sync
        % the frequency, then update accordingly
        % 
        function obj = set_goal_target(obj, goal_target, rho, delta_time)

            obj.mode = "traveling"; % set the mode to traveling
            % mode_history will be recorded automatically in energy_calculation()
            
            % goal target theta value should be the direction that current position to goal target
            goal_target_direction = atan2(goal_target(2) - obj.position(2), goal_target(1) - obj.position(1));
            obj.goal_target = [goal_target(1), goal_target(2), goal_target_direction];

            obj.rho = rho;
            distance_move = sqrt((obj.goal_target(1) - obj.position(1))^2 + (obj.goal_target(2) - obj.position(2))^2);
            distance_turn = wrapToPi(goal_target_direction - obj.position(3));
            % Record start of traveling phase
            phase_entry = struct();
            phase_entry.phase_type = "traveling";
            phase_entry.rho = rho;
            phase_entry.distance = distance_move;
            phase_entry.energy_start = obj.e_total_history(end);
            phase_entry.battery_start = obj.battery_percentage_history(end);
            phase_entry.duration = 0; % Will be updated at end
            phase_entry.start_time = length(obj.e_total_history); % Time step index
            phase_entry.tau = [];
            phase_entry.planning_time = [];
            phase_entry.energy_end = [];
            phase_entry.battery_end = [];
            phase_entry.energy_consumed = [];
            phase_entry.battery_delta = [];
            phase_entry.end_time = [];
            obj.phase_data = [obj.phase_data, phase_entry];

            

            % set turn speed fixed at 10 rad/s
            turn_time = abs(distance_turn) / Agent.ROTATION_SPEED;
            move_time = rho - turn_time;

             
            time_stamp = 0:delta_time:rho;

            N = length(time_stamp);

            % Initial arrays
            lin_vel_plan = zeros(1, N);
            ang_vel_plan = zeros(1, N);
            lin_acc_plan = zeros(1, N);
            ang_acc_plan = zeros(1, N);
            % Phase 1: Turn
            turn_end_idx = round(turn_time / delta_time) + 1;
            if abs(distance_turn) > 0.1 && turn_time > 0
                if turn_end_idx > N
                    turn_end_idx = N;
                end
                % account for discretization error, actual steps = turn_end_idx - 1
                actual_steps = turn_end_idx - 1;
                if actual_steps > 0
                    exact_ang_vel = distance_turn / (actual_steps * delta_time);
                else
                    exact_ang_vel = 0;
                end
                ang_vel_plan(1:actual_steps) = exact_ang_vel;
                ang_acc_plan(1:actual_steps) = 0;
                lin_vel_plan(1:actual_steps) = 0;
                lin_acc_plan(1:actual_steps) = 0;
            else
                turn_end_idx = 1;
            end
            % Phase 2: Move
            if distance_move > 0.05 && move_time > 0
                move_start_idx = turn_end_idx +1;
                [v_move_profile, u_move_profile] = Agent.generate_energy_optimal_velocity_fmincon(distance_move, move_time, delta_time);
                move_N = length(v_move_profile);
                for i = 1:move_N
                    idx = move_start_idx + i - 1;
                    if idx <= N
                        lin_vel_plan(idx) = v_move_profile(i);
                        lin_acc_plan(idx) = u_move_profile(i);
                    end
                end
            else
                move_start_idx = turn_end_idx +1;
                lin_vel_plan(move_start_idx:N) = 0;
                lin_acc_plan(move_start_idx:N) = 0;
            end

            % store in the trajectory
            obj.trajectory = struct();
            obj.trajectory.time = time_stamp;
            obj.trajectory.lin_vel = lin_vel_plan;
            obj.trajectory.lin_acc = lin_acc_plan;
            obj.trajectory.ang_vel = ang_vel_plan;
            obj.trajectory.ang_acc = ang_acc_plan;

            obj.current_time = 0; % Initialize current time counter
            
        end

        % set_dwelling_time Put the agent into dwelling mode for tau seconds
        %   obj = obj.set_dwelling_time(tau)
        %
        % Inputs:
        %   tau - dwelling time (seconds)
        function obj = set_dwelling_time(obj, tau)
                obj.mode = "dwelling"; % set the mode to dwelling
                % mode_history will be recorded automatically in energy_calculation()

                obj.tau = tau;
                obj.dwelling_time_remaining = tau;
                
                % Record start of dwelling phase
                phase_entry = struct();
                phase_entry.phase_type = "dwelling";
                phase_entry.tau = tau;
                phase_entry.distance = 0;
                phase_entry.energy_start = obj.e_total_history(end);
                phase_entry.battery_start = obj.battery_percentage_history(end);
                phase_entry.duration = 0; % Will be updated at end
                phase_entry.start_time = length(obj.e_total_history); % Time step index
                phase_entry.rho = [];
                phase_entry.planning_time = [];
                phase_entry.energy_end = [];
                phase_entry.battery_end = [];
                phase_entry.energy_consumed = [];
                phase_entry.battery_delta = [];
                phase_entry.end_time = [];
                obj.phase_data = [obj.phase_data, phase_entry];
                
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


        function obj = motion_energy_calculation(obj)
            % Physics-based motion power calculation
            % P_move = P_0 + α·v + γ·u²
            % where v = lin_velocity, u = lin_acceleration
            
            % Ensure lin_velocity and lin_acceleration are defined
            if isempty(obj.lin_velocity) || isnan(obj.lin_velocity)
                obj.lin_velocity = 0;
            end
            if isempty(obj.lin_acceleration) || isnan(obj.lin_acceleration)
                obj.lin_acceleration = 0;
            end
            
            % Calculate motion power: P = P_0 + α·v + γ·u²
            obj.motion_power = Agent.P0_BASE_MOTION + ...
                               Agent.ALPHA_VELOCITY * obj.lin_velocity + ...
                               Agent.GAMMA_ACCELERATION * (obj.lin_acceleration^2);
            
            % Ensure non-negative power
            if obj.motion_power < 0
                obj.motion_power = 0;
            end
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

            % Total enrgy (with efficiency loss)
            obj.e_total = (p_total * delta_time) + obj.e_total_history(end);
            obj.e_total_history = [obj.e_total_history,obj.e_total];

            %% Obtain true SOC through coulomb counting  
            % Update battery state
            soc_current = obj.soc_history(end);
            ocv_polynomial = polyval(flip(Agent.k_dis), soc_current * 20.0);
            ocv_current = Agent.BAT_MIN_V + (ocv_polynomial - Agent.OCV_EMPTY) * Agent.SCALE_FACTOR;

            i_current = (ocv_current - sqrt(ocv_current^2 - 4 * Agent.R_INTERNAL * p_total)) /(2 * Agent.R_INTERNAL);
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
            % Store previous mode before recording current mode (for logic checks)
            
            
            % Record current mode at this time step (matches e_total_history)
            obj.mode_history = [obj.mode_history, obj.mode];
            
            switch obj.mode
                case "power_outage"
                    % Robot stops moving when battery is below cutoff voltage
                    obj.lin_velocity = 0;
                    obj.lin_acceleration = 0;
                    obj.ang_velocity = 0;
                    obj.ang_acceleration = 0;

                case "traveling"
                    % find the index in the trajectory
                    if isempty(obj.trajectory) || isempty(obj.trajectory.time)
                        obj.lin_velocity = 0;
                        obj.lin_acceleration = 0;
                        obj.ang_velocity = 0;
                        obj.ang_acceleration = 0;
                    else
                        time_array = obj.trajectory.time;
                        idx = round(obj.current_time / delta_time)+1;
                        if idx < 1
                            idx = 1;
                        elseif idx > length(time_array)
                            idx = length(time_array);
                        end
                        v_lin = obj.trajectory.lin_vel(idx);
                        u_lin = obj.trajectory.lin_acc(idx);
                        w_ang = obj.trajectory.ang_vel(idx);
                        u_ang = obj.trajectory.ang_acc(idx);

                        obj.lin_velocity = v_lin;
                        obj.lin_acceleration = u_lin;
                        obj.ang_velocity = w_ang;
                        obj.ang_acceleration = u_ang;
                        % using unicycle kinematics model to update position
                        theta_current = obj.position(3);
                        obj.position(1) = obj.position(1) + v_lin * cos(theta_current) * delta_time;
                        obj.position(2) = obj.position(2) + v_lin * sin(theta_current) * delta_time;
                        obj.position(3) = obj.position(3) + w_ang * delta_time;
                    end
                    obj.current_time = obj.current_time + delta_time;
                    obj.energy_calculation(delta_time);
                    if obj.battery_percentage <= 10
                        obj.mode = "power_outage";
                        return; 
                    end
                    % check trajectory content, dump out every detail then
                    % swith mode. (abandoned time and distance checked)
                    % Count down travel time
                    distance_to_goal = norm(obj.position(1:2) - obj.goal_target(1:2));
                    if distance_to_goal <= 0.05 || obj.current_time >= obj.rho
                        obj.position = obj.goal_target;
                        
                        % Finalize previous traveling phase
                        if ~isempty(obj.phase_data) && obj.phase_data(end).phase_type == "traveling"
                            obj.phase_data(end).energy_end = obj.e_total_history(end);
                            obj.phase_data(end).battery_end = obj.battery_percentage_history(end);
                            obj.phase_data(end).energy_consumed = obj.phase_data(end).energy_end - obj.phase_data(end).energy_start;
                            obj.phase_data(end).battery_delta = obj.phase_data(end).battery_end - obj.phase_data(end).battery_start;
                            obj.phase_data(end).duration = obj.current_time; % Actual travel duration
                            obj.phase_data(end).end_time = length(obj.e_total_history);
                        end
                        
                        obj.mode = "planning";
                        obj.planning_time = rand(); % random planning time between 0-1s
                        obj.planning_time_remaining = obj.planning_time;
                        
                        % Record start of planning phase
                        phase_entry = struct();
                        phase_entry.phase_type = "planning";
                        phase_entry.planning_time = obj.planning_time;
                        phase_entry.distance = 0;
                        phase_entry.energy_start = obj.e_total_history(end);
                        phase_entry.battery_start = obj.battery_percentage_history(end);
                        phase_entry.duration = 0;
                        phase_entry.start_time = length(obj.e_total_history);
                        phase_entry.rho = [];
                        phase_entry.tau = [];
                        phase_entry.energy_end = [];
                        phase_entry.battery_end = [];
                        phase_entry.energy_consumed = [];
                        phase_entry.battery_delta = [];
                        phase_entry.end_time = [];
                        obj.phase_data = [obj.phase_data, phase_entry];
                    end

                case "dwelling"
                    % stay at target position
                    obj.lin_velocity = 0;
                    obj.lin_acceleration = 0;
                    obj.ang_velocity = 0;
                    obj.ang_acceleration = 0;

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
                        % Finalize previous dwelling phase
                        if ~isempty(obj.phase_data) && obj.phase_data(end).phase_type == "dwelling"
                            obj.phase_data(end).energy_end = obj.e_total_history(end);
                            obj.phase_data(end).battery_end = obj.battery_percentage_history(end);
                            obj.phase_data(end).energy_consumed = obj.phase_data(end).energy_end - obj.phase_data(end).energy_start;
                            obj.phase_data(end).battery_delta = obj.phase_data(end).battery_end - obj.phase_data(end).battery_start;
                            obj.phase_data(end).duration = obj.tau - obj.dwelling_time_remaining; % Actual dwelling duration
                            obj.phase_data(end).end_time = length(obj.e_total_history);
                        end
                        
                        % Find which target we're currently at
                        obj.mode = "planning";
                        obj.planning_time = rand(); % random planning time between 0-1s
                        obj.planning_time_remaining = obj.planning_time;
                        
                        % Record start of planning phase
                        phase_entry = struct();
                        phase_entry.phase_type = "planning";
                        phase_entry.planning_time = obj.planning_time;
                        phase_entry.distance = 0;
                        phase_entry.energy_start = obj.e_total_history(end);
                        phase_entry.battery_start = obj.battery_percentage_history(end);
                        phase_entry.duration = 0;
                        phase_entry.start_time = length(obj.e_total_history);
                        phase_entry.rho = [];
                        phase_entry.tau = [];
                        phase_entry.energy_end = [];
                        phase_entry.battery_end = [];
                        phase_entry.energy_consumed = [];
                        phase_entry.battery_delta = [];
                        phase_entry.end_time = [];
                        obj.phase_data = [obj.phase_data, phase_entry];
                    end

                case "idle" % defalut mode
                    % do nothing, waiting for new target assignment
                    obj.lin_velocity = 0;
                    obj.lin_acceleration = 0;
                    obj.ang_velocity = 0;
                    obj.ang_acceleration = 0;
                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);


                case "planning"
                    % embedded the decision making process here
                    obj.lin_velocity = 0;
                    obj.lin_acceleration = 0;
                    obj.ang_velocity = 0;
                    obj.ang_acceleration = 0;
                    obj.planning_time_remaining = obj.planning_time_remaining - delta_time;

                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);
                    if obj.battery_percentage <= 10
                        obj.mode = "power_outage";
                        return; 
                    end

                    % Check if planning time is finished
                    if obj.planning_time_remaining <= 0
                        % Finalize planning phase
                        if ~isempty(obj.phase_data) && obj.phase_data(end).phase_type == "planning"
                            obj.phase_data(end).energy_end = obj.e_total_history(end);
                            obj.phase_data(end).battery_end = obj.battery_percentage_history(end);
                            obj.phase_data(end).energy_consumed = obj.phase_data(end).energy_end - obj.phase_data(end).energy_start;
                            obj.phase_data(end).battery_delta = obj.phase_data(end).battery_end - obj.phase_data(end).battery_start;
                            obj.phase_data(end).duration = obj.planning_time - obj.planning_time_remaining; % Actual planning duration
                            obj.phase_data(end).end_time = length(obj.e_total_history);
                        end
                        
                        for i = length(obj.mode_history):-1:1
                            if obj.mode_history(i) == "dwelling" || obj.mode_history(i) == "traveling"
                                previous_mode = obj.mode_history(i);
                                break;
                            end
                        end
                        if previous_mode == "dwelling"
                            % mode_history will be recorded automatically in energy_calculation()
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
                            goal_idx_list = [];
                            J_opt_list = [];
                            opt_values_list = [];
                            J_uncertainty_list = [];
                            E_bar_list = [];
                            
                            for i = 1:length(avalible_indices)
                                % try all possible goal target 
                                goal_target_idx = avalible_indices(i);
                                % provide lower bound based to make sure agent perform with maximum velocity and acceleration
                                goal_target_direction = atan2(obj.available_targets(goal_target_idx).position(2) - obj.position(2), obj.available_targets(goal_target_idx).position(1) - obj.position(1));
                                distance_move = sqrt((obj.available_targets(goal_target_idx).position(1) - obj.position(1))^2 + (obj.available_targets(goal_target_idx).position(2) - obj.position(2))^2);
                                distance_turn = wrapToPi(goal_target_direction - obj.position(3));
                                turn_time_capacity = abs(distance_turn) / Agent.ROTATION_SPEED;
                                % add this in the report
                                % distance check determine triangle or trapezoidal profile
                                distance_check = Agent.V_MAX ^ 2 / Agent.U_MAX;
                                if distance_move > distance_check
                                    move_time_capacity = distance_move / Agent.V_MAX + Agent.V_MAX / Agent.U_MAX;
                                else
                                    move_time_capacity = 2 * sqrt(distance_move / Agent.U_MAX);
                                end
                                % add this multiplier to change lb and create better data to train
                                % this prevent the agent to use max energy data to travel
                                multiplier = 1.0;
                                x0 = [5; 1];
                                lb = [(turn_time_capacity + move_time_capacity) * multiplier; 0];
                                ub = [inf; inf];
                                % find the optimal decision through fmincon, (objective travel gives the objective value of 3 events horizon) See Function below
                                [x_opt,J_opt] = fmincon(@(x) Agent.objective_travel(x(1), x(2),  ...
                                    r0i, Ai, Bi,  all_target_indices, goal_target_idx) + obj.K * obj.objective_travel_energy(x(1), x(2), distance_move), ...
                                    x0,[],[],[],[],lb, ub);
                                J_uncertainty = Agent.objective_travel(x_opt(1), x_opt(2),  ...
                                    r0i, Ai, Bi,  all_target_indices, goal_target_idx);
                                E_bar = obj.objective_travel_energy(x_opt(1), x_opt(2), distance_move);
                                goal_idx_list = [goal_idx_list, goal_target_idx];
                                J_opt_list = [J_opt_list, J_opt];
                                opt_values_list = [opt_values_list, x_opt(1)];
                                J_uncertainty_list = [J_uncertainty_list, J_uncertainty];
                                E_bar_list = [E_bar_list, E_bar];
                                if J_temp > J_opt
                                    J_temp = J_opt;
                                    % Only execute the first event with target id and correspond rho value
                                    target_idx = goal_target_idx;
                                    new_rho = x_opt(1);
                                end
                            end
                            % Log the receding horizon decision
                            RH_entry = struct();
                            RH_entry.optimal = "travel";
                            RH_entry.current_idx = current_target_idx;
                            RH_entry.goal_idx = goal_idx_list;
                            RH_entry.J_opt = J_opt_list;
                            RH_entry.opt_values = opt_values_list;
                            RH_entry.J_uncertainty = J_uncertainty_list;
                            RH_entry.E_bar = E_bar_list;
                            
                            obj.RH_log = [obj.RH_log, RH_entry];

                            obj.set_goal_target([obj.available_targets(target_idx).position,0], new_rho, delta_time)

                        elseif previous_mode == "traveling"
                            % mode_history will be recorded automatically in energy_calculation()
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
                            goal_idx_list = [];
                            J_opt_list = [];
                            opt_values_list = [];
                            J_uncertainty_list = [];
                            E_bar_list = [];
                            
                            for i = 1:length(avalible_indices)
                                goal_target_idx = avalible_indices(i);
                                distance_move = sqrt((obj.available_targets(goal_target_idx).position(1) - obj.position(1))^2 + (obj.available_targets(goal_target_idx).position(2) - obj.position(2))^2);
                                % try all posible target
                                x0 = [5; 1; 1];
                                lb = [0; 0; 0];
                                ub = [inf; inf; inf];
                                % find the optimal decision through fmincon (objective dwell gives the objective value of 2 events horizon) See Function below)
                                [x_opt,J_opt] = fmincon(@(x) Agent.objective_dwell(x(1), x(2), x(3), ...
                                    r0i, Ai, Bi, ...
                                    current_target_idx, all_target_indices, goal_target_idx) + obj.K * obj.objective_dwell_energy(x(1), x(2), x(3), distance_move), ...
                                    x0, [], [], [], [], lb, ub);
                                J_uncertainty = Agent.objective_dwell(x_opt(1), x_opt(2), x_opt(3), ...
                                    r0i, Ai, Bi, ...
                                    current_target_idx, all_target_indices, goal_target_idx);
                                E_bar = obj.objective_dwell_energy(x_opt(1), x_opt(2), x_opt(3), distance_move);
                                goal_idx_list = [goal_idx_list, goal_target_idx];
                                J_opt_list = [J_opt_list, J_opt];
                                opt_values_list = [opt_values_list, x_opt(1)];
                                J_uncertainty_list = [J_uncertainty_list, J_uncertainty];
                                E_bar_list = [E_bar_list, E_bar];
                                if J_temp > J_opt
                                    J_temp = J_opt;
                                    % Only execute the first event with dwelling time tau
                                    new_tau = x_opt(1);
                                end
                            end

                            % Log the receding horizon decision
                            RH_entry = struct();
                            RH_entry.optimal = "dwell";
                            RH_entry.current_idx = current_target_idx;
                            RH_entry.goal_idx = goal_idx_list;
                            RH_entry.J_opt = J_opt_list;
                            RH_entry.opt_values = opt_values_list;
                            RH_entry.J_uncertainty = J_uncertainty_list;
                            RH_entry.E_bar = E_bar_list;
                            obj.RH_log = [obj.RH_log, RH_entry];
                            obj.set_dwelling_time(new_tau);
                        else
                            fprintf("Invalid previous mode: %s\n", previous_mode);
                        
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

        % export_phase_data Export phase data to CSV file
        %   obj.export_phase_data('phase_data.csv')
        %
        % Inputs:
        %   filename - string filename for CSV export
        function export_phase_data(obj, filename)
            % Export phase_data to CSV file
            if isempty(obj.phase_data)
                warning('No phase data to export');
                return;
            end
            
            % Convert struct array to table
            T = struct2table(obj.phase_data);
            writetable(T, filename);
            fprintf('Phase data exported to %s\n', filename);
        end

        % export_RH_log Export RH log to CSV file
        %   obj.export_RH_log('RH_log.csv')
        %
        % Inputs:
        %   filename - string filename for CSV export
        function export_RH_log(obj, filename)
            % Export RH log to CSV file
            if isempty(obj.RH_log)
                warning('No RH log to export');
                return;
            end
            
            % Convert struct array to table
            T = struct2table(obj.RH_log);
            writetable(T, filename);
            fprintf('RH log exported to %s\n', filename);
        end


        % Travel_Energy Evaluate travel-phase energy contribution
        %   E = Agent.Travel_Energy(duration, distance)
        %
        % Inputs:
        %   duration - travel duration
        %   distance - travel distance
        %
        % Output:
        %   E     - travel energy consumption
        function E = travel_energy_prediction(obj, duration, distance)
            E = predict(obj.travel_predict, [duration, distance]);
        end

        % Dwelling_Energy Evaluate dwelling-phase energy contribution
        %   E = Agent.Dwelling_Energy(duration, distance)
        %
        % Inputs:
        %   duration - dwelling duration
        %   distance - dwelling distance
        %   distance - defalut to 0
        %
        % Output:
        %   E     - dwelling energy consumption
        function E = dwelling_energy_prediction(obj, duration, distance)
            E = predict(obj.dwell_predict, [duration, distance]);
        end

        % Planning_Energy Evaluate planning-phase energy contribution
        %   E = Agent.Planning_Energy(duration, distance)
        %
        % Inputs:
        %   duration - travel duration
        %   distance - travel distance
        %
        % Output:
        %   E     - planning energy consumption
        function E = planning_energy_prediction(obj, duration, distance)
            E = predict(obj.plan_predict, [duration, distance]);
        end


        %   E_bar = obj.objective_dwell_energy(tau1, rho1, tau2, distance)
        %
        % Inputs:
        %   tau1        - first dwell duration
        %   rho1        - travel duration
        %   tau2        - second dwell duration
        %   distance    - travel distance
        %
        % Output:
        %   E_bar       - average energy consumption over the combined horizon
        function E_bar = objective_dwell_energy(obj, tau1, rho1 , tau2, distance)
            % consist with dwell->travel->dwell
            E_plan = obj.planning_energy_prediction(0.8, 0);
            E_dwell = obj.dwelling_energy_prediction(tau1, 0);
            E_travel = obj.travel_energy_prediction(rho1, distance);
            E_dwell2 = obj.dwelling_energy_prediction(tau2, 0);
            % TODO: in case i forgot i change this
            % combine (take out event time for now)
            E_bar = (E_plan + E_dwell + E_travel + E_dwell2) / (0.8 + tau1 + rho1 + tau2);

        end

        %   E_bar = obj.objective_dwell_energy(tau1, rho1, distance)
        %
        % Inputs:
        %   tau1        - first dwell duration
        %   rho1        - travel duration
        %   tau2        - second dwell duration
        %   distance    - travel distance
        %
        % Output:
        %   E_bar       - average energy consumption over the combined horizon
        function E_bar = objective_travel_energy(obj, rho1 , tau1, distance)
            % consist with dwell->travel->dwell
            E_plan = obj.planning_energy_prediction(0.8, 0);
            E_travel = obj.travel_energy_prediction(rho1, distance);
            E_dwell = obj.dwelling_energy_prediction(tau1, 0);
            % TODO: in case i forgot i change this
            % combine (take out event time for now)
            E_bar = (E_plan + E_travel + E_dwell) / (0.8 + rho1 + tau1);

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
                J = r0_sum*tau + 0.5*(A_sum - B)*tau^2;
            else
                J = (r0_sum - r0)*tau + 0.5*(A_sum - A)*tau^2 + 0.5 * r0 * t_sat;
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
            J = r0_sum*rho + 0.5*A_sum*rho^2;
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
                    r0i(current_target_idx) = Agent.uncertainty_mon(r0i(current_target_idx), Ai(current_target_idx),Bi(current_target_idx), tau1);
                else
                    r0i(i) = Agent.uncertainty_unmon(r0i(i), Ai(i), tau1);
                end
            end
            r0_sum = sum(r0i);

            %% first travel time
            J_first_travel = Agent.Travel(r0_sum, A_sum, rho1);

            % update r0_sum for next event // all targets are not being monitored
            for i = 1: length(all_target_indices)
                r0i(i) = Agent.uncertainty_unmon(r0i(i), Ai(i), rho1);
            end
            %% Second dwell time 
            J_second_dwell = Agent.Dwell(r0_sum, A_sum, r0i(goal_target_idx),Ai(goal_target_idx),Bi(goal_target_idx), tau2);


            % combine 
            J = (J_first_dwell + J_first_travel + J_second_dwell) / (tau1 + rho1 + tau2);

        end

                % objective_dwell Horizon cost for dwell-travel-dwell schedule
        
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

        

        function [v_profile, u_profile] = generate_energy_optimal_velocity_fmincon(distance, time_available, delta_time)
            % Solve energy minimization by optimizing u(t) directly
            % Then compute v(t) = ∫ u(τ) dτ
            
            % ===== STEP 1: Energy Model Parameters =====
            P0 = Agent.P0_BASE_MOTION;
            alpha = Agent.ALPHA_VELOCITY;
            gamma = Agent.GAMMA_ACCELERATION;
            T = time_available;
            D = distance;
            v_max = Agent.V_MAX;
            u_max = Agent.U_MAX;
            
            % Discretize time
            
            time_array = 0:delta_time:T;
            N = length(time_array);
            
            % ===== STEP 2: Initial Guess for u(t) =====
            % Guess: constant acceleration/deceleration profile
            % u(t) that gives v(0) = 0, v(T) = 0, and covers distance D
            % For a simple guess: u(t) = a for first half, -a for second half
            u0_guess = zeros(N, 1);
            % Simple trapezoidal guess
            u0_guess(1:round(N/2)) = 2*D / (T^2);      % Accelerate
            u0_guess(round(N/2)+1:end) = -2*D / (T^2); % Decelerate
            
            % ===== STEP 3: Objective Function - Minimize Total Energy =====
            function E_total = energy_objective(u)
                % u is a column vector [u₁, u₂, ..., uₙ] representing acceleration
                
                % Compute v(t) from u(t) by integration: v(t) = ∫₀^t u(τ) dτ
                v = zeros(N, 1);
                v(1) = 0;  % v(0) = 0 (boundary condition)
                for i = 2:N
                    % Trapezoidal integration: v(i) = v(i-1) + (u(i-1) + u(i))/2 * Δt
                    v(i) = v(i-1) + (u(i-1) + u(i)) / 2 * delta_time;
                end
                
                % Ensure non-negative velocity
                v = max(0, v);
                
                % ===== EXPLICIT ENERGY FUNCTION CALCULATION =====
                % For each time step: P(tᵢ) = P₀ + α·vᵢ + γ·uᵢ²
                P = zeros(N, 1);
                for i = 1:N
                    P(i) = P0 + alpha*v(i) + gamma*(u(i)^2);
                end
                
                % ===== TOTAL ENERGY: E = Σ P(tᵢ) · Δt =====
                E_total = sum(P) * delta_time;
            end
            
            % ===== STEP 4: Constraints =====
            function [c, ceq] = constraints(u)
                % Compute v(t) from u(t) by integration
                v = zeros(N, 1);
                v(1) = 0;  % v(0) = 0
                for i = 2:N
                    v(i) = v(i-1) + (u(i-1) + u(i)) / 2 * delta_time;
                end
                v = max(0, v);  % Non-negativity
                
                % Constraint 1: Distance constraint ∫₀^T v(t) dt = D
                distance_constraint = sum(v) * delta_time - D;
                
                % Constraint 2: Final velocity v(T) = 0
                % v(T) = ∫₀^T u(τ) dτ = 0
                final_velocity_constraint = v(N);  % v(T) = 0

                % Constraint 3: Maximum velocity constraint v(t) <= v_max

                
                ceq = [distance_constraint; final_velocity_constraint];
                c = v - v_max;  %Only velocity constraint ; accelaration constraint in lb and ub
            end
            
            % ===== STEP 5: Bounds ===== 
            lb = -u_max * ones(N, 1);  % acceleration lower bound
            ub = u_max * ones(N, 1);   % acceleration upper bound
            
            % Note: No need for linear equality constraints on u(0) since v(0) = 0 is handled in integration
            
            % ===== STEP 6: Solve Optimization =====
            options = optimoptions('fmincon', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point', ...
                'MaxIterations', 1000, ...
                'OptimalityTolerance', 1e-6);
            
            u_opt = fmincon(@energy_objective, u0_guess, [], [], [], [], lb, ub, @constraints, options);
            
            % ===== STEP 7: Compute v(t) from optimal u(t) =====
            u_profile = u_opt';
            
            % Integrate to get velocity: v(t) = ∫ u(τ) dτ
            v_profile = zeros(1, N);
            v_profile(1) = 0;  % v(0) = 0
            for i = 2:N
                v_profile(i) = v_profile(i-1) + (u_profile(i-1) + u_profile(i)) / 2 * delta_time;
            end
            
            % Ensure boundary conditions
            v_profile(1) = 0;
            v_profile(end) = 0;  % Should be satisfied by constraint, but ensure
            
            % Ensure non-negative velocity
            v_profile = max(0, v_profile);
            
            % Verify distance and energy
            distance_check = sum(v_profile) * delta_time;
            P_final = P0 + alpha*v_profile + gamma*(u_profile.^2);
            E_final = sum(P_final) * delta_time;
            % fprintf('Distance: desired=%.4f, actual=%.4f\n', D, distance_check);
            % fprintf('Total energy: E=%.4f J\n', E_final);
        end

    end
end
