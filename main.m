% main.m
% This is the main file for the simulation
% For now, i will only create one target to see the simulation and targets behavior works as expected
% for the simulation, we will think of a way to visulize the simulation in a 2D plot
% Persistent Monitoring System Simulation
% Main script for the persistent monitoring simulation.
%
% Syntax:
%   Run this script from the MATLAB command window or editor:
%     main
%
% Description:
%   Initializes targets and agents, creates the visualization, and runs the
%   simulation loop with time resolution `delta_time` until time `T`.
%
% Parameters (defined below):
%   T         - Total simulation time (seconds)
%   delta_time - Simulation time step (seconds)
%
% Example:
%   main  % runs the simulation with the configured parameters

clear;clc;

T = 4000;
delta_time = 0.1;

% Create multiple target objects
target1 = Target(1, [0, 0], 10, 1.0, 10);
target2 = Target(2, [10, 10], 15, 2.0, 10);
target3 = Target(3, [0, 10], 12, 3.0, 10);
target4 = Target(4, [10, 0], 13, 4.0, 10);



% Create agent object
agent1 = Agent(1, [0 0 0]);
% set available targets for agent1
agent1.set_available_targets([target1, target2, target3, target4]); % Pass target objects
agent1.set_goal_target([10, 10, 0], 2,delta_time); % rho = 5 (travel time)
% Create visualizer
viz = SimulationVisualizer();

% Initialize plot with all targets and agent
viz.initialize_plot([target1, target2, target3, target4], [agent1]);

% Simulation loop with visualization
for i = 0:delta_time:T
    fprintf('Time step %d:\n', i);
    
    % Update each target (test monitor mode on target1)
    [R1, objective1, global_objective1] = target1.update_state(delta_time, agent1.position);
    [R2, objective2, global_objective2] = target2.update_state(delta_time, agent1.position);
    [R3, objective3, global_objective3] = target3.update_state(delta_time, agent1.position);
    [R4, objective4, global_objective4] = target4.update_state(delta_time, agent1.position);
    
    
    % Update agent
    agent1.update_state(delta_time);
    
    
    % Debug: Print agent state
    fprintf('Agent: pos=[%.1f,%.1f], mode=%s, vel=[%.1f,%.1f]\n', ...
            agent1.position(1), agent1.position(2), agent1.mode, ...
            agent1.lin_velocity, agent1.ang_velocity);
    
    % Update visualization
    % make sure all targets included
    viz.update_visualization([target1, target2, target3, target4], [agent1], i);
    
    % Print values to console
    fprintf('Target 1: R=%.1f, objective=%.1f, global_objective=%.1f \n', R1, objective1, global_objective1);
    fprintf('Target 2: R=%.1f, objective=%.1f, global_objective=%.1f \n', R2, objective2, global_objective2);
    fprintf('Target 3: R=%.1f, objective=%.1f, global_objective=%.1f \n', R3, objective3, global_objective3);
    fprintf('\n');
    
    % Pause for animation effect
    pause(0.01);
end


