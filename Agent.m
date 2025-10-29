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
        position

        mode
        mode_history

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
        travel_time_remaining   
        
        % ===== ENERGY PROPERTIES =====
        e_total

        % Energy history
        e_cpu_history
        e_acc_history
        e_mot_history
        e_total_history

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
        BAT_CUTOFF_V   = 10.8;  % V (BMS cutoff)
        BATTERY_E_Wh   = 26.0;  % Wh (capacity)

        % ===== MOTION POWER =====
        % Keep Create3 team's measurements as they are based on actual hardware
        % fixed velocity
        % include velocity variation(v , u)
        DRIVE_CURRENT  = 0.526;  % A
        IDLE_CURRENT   = 0.404;  % A
        MOT_MOVE = Agent.BAT_MON_V * Agent.DRIVE_CURRENT;   % 7.57 W
        MOT_IDLE = Agent.BAT_MON_V * Agent.IDLE_CURRENT;    % 5.82 W

        % ===== EFFICIENCY =====
        ETA_CONV = 0.90;  % DC-DC & wiring loss

        % ===== BATTERY PERCENTAGE CONVERSION =====
        SOC_TO_PERCENTAGE = 10000;      % Multiply SOC by 10000 (0.01% resolution)
        PERCENTAGE_TO_SOC = 0.0001;     % Divide percentage by 10000

        % ===== POLYNOMIAL COEFFICIENTS =====
        % 17th-order polynomial for LMO discharge (Somakettarin & Funaki 2017)
        k_dis = [3.0016, 2.0082e-1, -5.0440e-2, 1.1287e-2, -1.7962e-3, ...
                1.9382e-4, -1.4444e-5, 7.6498e-7, -2.9500e-8, ...
                8.4261e-10, -1.8003e-11, 2.8847e-13, -3.4474e-15, ...
                3.0256e-17, -1.8926e-19, 7.9827e-22, -2.0344e-24, 2.3659e-27];
        
        % SOC look-up table / information
        OCV_EMPTY = polyval(flip(Agent.k_dis), 0 * 20.0); % 0% SOC
        OCV_FULL  = polyval(flip(Agent.k_dis), 1 * 20.0); % 100% SOC
        OCV_MIN = min(Agent.BAT_CUTOFF_V, Agent.OCV_EMPTY);
        OCV_MAX = max(Agent.BAT_MAX_V, Agent.OCV_FULL);
        SCALE_FACTOR = (Agent.BAT_MAX_V - Agent.BAT_CUTOFF_V) / (Agent.OCV_MAX - Agent.OCV_MIN);

    end    
    methods
        % Agent constructor
        %   obj = Agent(index, position)
        %
        % Inputs:
        %   index    - integer identifier for the agent
        %   position - 1x2 numeric vector specifying initial [x y] position
        function obj = Agent(index, position)
            obj.index = index;
            obj.position = position;    
            obj.goal_target = [0,0]; % default goal target is [0,0]
            obj.mode = "idle"; % default mode is idle
            obj.velocity = [0, 0];
            obj.dwelling_time_remaining = 0;
            obj.travel_time_remaining = 0;
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

            obj.battery_percentage = 100; % start with full battery
            obj.battery_percentage_history = [100]; % start with full battery

            obj.voltage_history = [Agent.BAT_MAX_V]; % start with max voltage
        end

        % set_goal_target Assign a new target and travel time (rho)
        %   obj = obj.set_goal_target(goal_target, rho)
        %
        % Inputs:
        %   goal_target - 1x2 numeric vector of target [x y]
        %   rho         - desired travel time to reach the goal (seconds)
        function obj = set_goal_target(obj, goal_target, rho)

            obj.mode = "traveling"; % set the mode to traveling
            obj.mode_history = [obj.mode_history, "traveling"];

            obj.goal_target = goal_target;
            obj.goal_target_history = [obj.goal_target_history; goal_target];

            obj.rho = rho;
            obj.rho_history = [obj.rho_history; rho];

            obj.travel_time_remaining = rho; % Initialize travel time counter
            % calculate the distance to the goal target
            distance = norm(obj.position - obj.goal_target);
            % calculate the velocity to reach target in rho time
            if distance > 0
                obj.velocity = (obj.goal_target - obj.position) / obj.rho;
            else
                obj.velocity = [0, 0];
            end
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
                obj.velocity = [0, 0]; % stop moving
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
                if norm(obj.position - obj.available_targets(i).position) < 0.2
                    current_target_idx = i;
                    break;
                end
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
        function energy_calculation(obj, delta_time)
            % Calculate OU noise
            ou_noise = obj.ou_noise_history(end) + obj.theta_ou * (obj.mu_ou - obj.ou_noise_history(end)) * delta_time + obj.sigma_ou * sqrt(delta_time) * randn();
            obj.ou_noise_history = [obj.ou_noise_history, ou_noise];
            % Calculate insntantaneous power with OU noise (+- 20% variation)
            switch obj.mode
                case "planning"
                    p_cpu = obj.CPU_PLAN * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_ACTIVE * (1 + 0.2 * ou_noise);
                    p_mot = obj.MOT_IDLE * (1 + 0.2 * ou_noise);
                case "traveling"
                    p_cpu = obj.CPU_MOVE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_ACTIVE * (1 + 0.2 * ou_noise);
                    p_mot = obj.MOT_MOVE * (1 + 0.2 * ou_noise);
                case "dwelling"
                    p_cpu = obj.CPU_IDLE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_STANDBY * (1 + 0.2 * ou_noise);
                    p_mot = obj.MOT_IDLE * (1 + 0.2 * ou_noise);
                case "idle"
                    p_cpu = obj.CPU_IDLE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_STANDBY * (1 + 0.2 * ou_noise);
                    p_mot = obj.MOT_IDLE * (1 + 0.2 * ou_noise);
                otherwise
                    p_cpu = obj.CPU_IDLE * (1 + 0.2 * ou_noise);
                    p_acc = obj.COMP_STANDBY * (1 + 0.2 * ou_noise);
                    p_mot = obj.MOT_IDLE * (1 + 0.2 * ou_noise);
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
            % TODO: Assess if using accumalate or instantaneous value(using accumalate value for now)
            obj.e_total = (p_total * delta_time) + obj.e_total_history(end);

            obj.e_total_history = [obj.e_total_history,obj.e_total];

            % Update battery state
            v_curent = obj.voltage_history(end);

            % Coulomb counting
            i_cpu = p_cpu / v_curent;
            i_acc = p_acc / v_curent;
            i_mot = p_mot / v_curent;

            % update cumulative charge consumed (Q = I * delta_time)
            obj.q_cpu_history = [obj.q_cpu_history, i_cpu * delta_time];
            obj.q_acc_history = [obj.q_acc_history, i_acc * delta_time];
            obj.q_mot_history = [obj.q_mot_history, i_mot * delta_time];
            % cumulative coulombs consumed (sum of history) and account for efficiency
            q_total_cumulative = (sum(obj.q_cpu_history) + sum(obj.q_acc_history) + sum(obj.q_mot_history)) / Agent.ETA_CONV;
            obj.q_total_history = [obj.q_total_history, q_total_cumulative];

            % Calculate total battery capacity in Coulombs
            % Q = E / V = (Battery energy Wh * 3600 s/h) / nominal voltage V
            Q_battery_C = (Agent.BATTERY_E_Wh * 3600) / Agent.BAT_MON_V;
            % Update State of Charge (SOC)
            soc_new = 1 - (q_total_cumulative / Q_battery_C);
            soc_new = max(0, min(1, soc_new)); % Clamp between 0 and 1
            % Convert SOC to percentage (0-100)
            obj.battery_percentage = soc_new * Agent.SOC_TO_PERCENTAGE / 100;
            obj.battery_percentage_history = [obj.battery_percentage_history, obj.battery_percentage];

            % calculate voltage form soc using polynominal relationship
            ocv_raw = polyval(flip(Agent.k_dis), soc_new * 20.0); % Convert SOC to percentage for polynomial
            voltage_new = Agent.BAT_CUTOFF_V + (ocv_raw - Agent.OCV_MIN) * Agent.SCALE_FACTOR;
            obj.voltage_history = [obj.voltage_history, voltage_new];



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
                    % update the agent position
                    obj.position = obj.position + obj.velocity * delta_time;

                    % update battery based on energy consumption
                    obj.energy_calculation(delta_time);
                    if obj.battery_percentage <= 10
                        obj.mode = "power_outage";
                        return; % is this needed? what can it do ?
                    end
                    
                    % Count down travel time
                    obj.travel_time_remaining = obj.travel_time_remaining - delta_time;
                    
                    % Check if reached target (trigger to dwelling mode)
                    distance = norm(obj.position - obj.goal_target);
                    if distance < 0.1  % Distance-based trigger
                        %obj.position = obj.goal_target; % snap to target
                        obj.mode = "planning";
                        obj.planning_time = rand(); % random planning time between 0-1s
                        obj.planning_time_remaining = obj.planning_time;

                    elseif obj.travel_time_remaining <= 0  % Time-based failsafe
                        %obj.position = obj.goal_target; % snap to target
                        obj.mode = "planning";
                        obj.planning_time = rand(); % random planning time between 0-1s
                        obj.planning_time_remaining = obj.planning_time;

                    end
                    
                case "dwelling"
                    % stay at target position
                    obj.position = obj.goal_target;
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
                        return; % is this needed? what can it do ?
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
                            obj.set_goal_target(obj.available_targets(target_idx).position, new_rho)

                            
                           
                            
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