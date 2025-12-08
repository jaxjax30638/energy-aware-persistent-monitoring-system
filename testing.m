rho = 20;
waypoints_turn = [0 0 0; 0 0 5];

waypoints_move = [0 0 5; 5 5 5];

trajectory_turn = waypointTrajectory(Waypoints=waypoints_turn,...
    SampleRate=1/delta_time,TimeOfArrival=[0, rho/5] );
trajectory_move = waypointTrajectory(Waypoints=waypoints_move,...
    SampleRate=1/delta_time,TimeOfArrival=[rho/5, rho] );
t_vec = 0: delta_time : rho;

for time_step = 0: delta_time : rho
    if time_step <= rho/5
        [position_traj, ~, velocity_traj, acceleration_traj, ~] = lookupPose(trajectory_turn, time_step);
    else
        [position_traj, ~, velocity_traj, acceleration_traj, ~] = lookupPose(trajectory_move, time_step );
    end
    fprintf('Time step %d: position = [%.1f, %.1f, %.1f], velocity = [%.1f, %.1f, %.1f], acceleration = [%.1f, %.1f, %.1f]\n', time_step, position_traj(1), position_traj(2), position_traj(3), velocity_traj(1), velocity_traj(2), velocity_traj(3), acceleration_traj(1), acceleration_traj(2), acceleration_traj(3));
end


