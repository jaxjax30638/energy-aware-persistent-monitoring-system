% train the energy prediction model

% The script:
% Input: duration, distance, phase type (traveling, dwelling, planning)
% Output: energy consumption

clear; clc;
close all;

data_dir = 'Energy_Training_Data/';

files = dir(fullfile(data_dir, 'data*.csv'));
if isempty(files)
    error('No data files found in the directory');
end

[~, idx] = sort({files.name});
files = files(idx);

data = [];
for i = 1:length(files)
    filepath = fullfile(data_dir, files(i).name);
    try
        data = readtable(filepath);
        fprintf('Loaded data from %s\n', files(i).name);
    catch ME
        warning('Error loading data from %s: %s', files(i).name, ME.message);
    end
end

if isempty(data)
    error('No data loaded');
end


% Extract features and labels
duration = data.duration;
distance = data.distance;
phase_type = categorical(data.phase_type);

% Extract targets
energy_consumed = data.energy_consumed;
battery_delta = data.battery_delta;

% Check for invalid data
% remove rows with:
% duration <= 0 
% no energy_consumed, no battery_delta
valid_idx = duration > 0 & ~isnan(energy_consumed) & ~isnan(battery_delta);

duration = duration(valid_idx);
distance = distance(valid_idx);
energy_consumed = energy_consumed(valid_idx);
battery_delta = battery_delta(valid_idx);
phase_type = phase_type(valid_idx);

% seperate datas into three different categories: traveling, dwelling, planning
traveling_idx = phase_type == "traveling";
dwelling_idx = phase_type == "dwelling";
planning_idx = phase_type == "planning";

% train the energy prediction model for each category
% Traveling model 
X_travel = [duration(traveling_idx), distance(traveling_idx)];
energy_model_travel = fitlm(X_travel, energy_consumed(traveling_idx));
battery_model_travel = fitlm(X_travel, battery_delta(traveling_idx));

fprintf('Energy Model: energy = %.4f + %.4f*duration + %.4f*distance\n', ...
    energy_model_travel.Coefficients.Estimate(1), ...
    energy_model_travel.Coefficients.Estimate(2), ...
    energy_model_travel.Coefficients.Estimate(3));
fprintf('R² = %.4f\n', energy_model_travel.Rsquared.Ordinary);


% Dwelling model
X_dwell = [duration(dwelling_idx), distance(dwelling_idx)];
energy_model_dwell = fitlm(X_dwell, energy_consumed(dwelling_idx));
battery_model_dwell = fitlm(X_dwell, battery_delta(dwelling_idx));

fprintf('Energy Model: energy = %.4f + %.4f*duration\n', ...
    energy_model_dwell.Coefficients.Estimate(1), ...
    energy_model_dwell.Coefficients.Estimate(2));
fprintf('R² = %.4f\n', energy_model_dwell.Rsquared.Ordinary);


% Planning model
X_plan = [duration(planning_idx), distance(planning_idx)];
energy_model_plan = fitlm(X_plan, energy_consumed(planning_idx));
battery_model_plan = fitlm(X_plan, battery_delta(planning_idx));

fprintf('Energy Model: energy = %.4f + %.4f*duration\n', ...
    energy_model_plan.Coefficients.Estimate(1), ...
    energy_model_plan.Coefficients.Estimate(2));
fprintf('R² = %.4f\n', energy_model_plan.Rsquared.Ordinary);


% Save the models
save('energy_model_travel.mat', 'energy_model_travel');
save('battery_model_travel.mat', 'battery_model_travel');
save('energy_model_dwell.mat', 'energy_model_dwell');
save('battery_model_dwell.mat', 'battery_model_dwell');
save('energy_model_plan.mat', 'energy_model_plan');
save('battery_model_plan.mat', 'battery_model_plan');