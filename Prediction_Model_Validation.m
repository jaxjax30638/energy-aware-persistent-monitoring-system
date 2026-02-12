% energy model validation
% in this part will load in a different set of data and plot the result
% show specific data for all three catogories 
% plot prediction data with same input and cross variance
clear; clc;
close all;
% load testing data set
data = readtable("data_testing.csv");

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

% pick one of data for each catogory to demonstrate
% Select one sample from each category for demonstration
% load model for comparison
idxTraveling = find(traveling_idx);
sampleTraveling = idxTraveling(10);
S = load("energy_model_travel.mat");
mdl_travel_eng = S.energy_model_travel;
S = load("battery_model_travel.mat");
mdl_travel_bat = S.battery_model_travel;

x_travel = [0, duration(sampleTraveling)];
y_travel = [0, distance(sampleTraveling)];

predict_eng_travel = predict(mdl_travel_eng, [duration(sampleTraveling), distance(sampleTraveling)]);
predict_bat_travel = predict(mdl_travel_bat, [duration(sampleTraveling), distance(sampleTraveling)]);

z_travel_eng = [0, energy_consumed(sampleTraveling)];
zhat_travel_eng = [0, predict_eng_travel];

z_travel_bat = [1, battery_delta(sampleTraveling)];
zhat_travel_bat = [1, predict_bat_travel];


idxDwelling = find(dwelling_idx);
sampleDwelling = idxDwelling(10);
S = load("energy_model_dwell.mat");
mdl_dwell_eng = S.energy_model_dwell;
S = load("battery_model_dwell.mat");
mdl_dwell_bat = S.battery_model_dwell;
x_dwell = [0, duration(sampleDwelling)];

predict_eng_dwell = predict(mdl_dwell_eng, [duration(sampleDwelling), distance(sampleDwelling)]);
predict_bat_dwell = predict(mdl_dwell_bat, [duration(sampleDwelling), distance(sampleDwelling)]);

y_dwell_eng = [0, energy_consumed(sampleDwelling)];
yhat_dwell_eng = [0, predict_eng_dwell];

y_dwell_bat = [1, battery_delta(sampleDwelling)];
yhat_dwell_bat = [1, predict_bat_dwell];


idxPlanning = find(planning_idx);
samplePlanning = idxPlanning(10);
S = load("energy_model_plan.mat");
mdl_plan_eng = S.energy_model_plan;
S = load("battery_model_plan.mat");
mdl_plan_bat = S.battery_model_plan;
x_plan= [0, duration(samplePlanning)];

predict_eng_plan = predict(mdl_plan_eng, [duration(samplePlanning), distance(samplePlanning)]);
predict_bat_plan = predict(mdl_plan_bat, [duration(samplePlanning), distance(samplePlanning)]);

y_plan_eng = [0, energy_consumed(samplePlanning)];
yhat_plan_eng = [0, predict_eng_plan];

y_plan_bat = [1, battery_delta(samplePlanning)];
yhat_plan_bat = [1, predict_bat_plan];


% Plot energy consumed vs duration vs distance for traveling (3D?)
figure;
plot3(x_travel, y_travel, z_travel_eng,'-o', 'LineWidth', 3, 'MarkerSize', 8 , 'MarkerFaceColor','b');

hold on;
plot3(x_travel, y_travel, zhat_travel_eng,'--s', 'LineWidth', 1, 'MarkerSize', 4 , 'MarkerFaceColor','r');

xlabel('duration');ylabel('Distance');zlabel('Energy Consumed');
title(sprintf('Sample %d: Phase = Traveling',sampleTraveling));
grid on 
hold on

view(45,25)
% plot energy vs duration for dwelling
figure;
plot(x_dwell, y_dwell_eng, '-o', 'LineWidth', 2, 'MarkerSize', 8 , 'MarkerFaceColor','b');
hold on;
plot(x_dwell, yhat_dwell_eng, '--s', 'LineWidth', 1, 'MarkerSize', 4 , 'MarkerFaceColor','r');


xlabel('Duration'); ylabel('Energy Consumed');
title(sprintf('Sample %d: Phase = Dwelling',sampleDwelling));
grid on
hold on
% plot energy consumed vs duration for planning
figure;
plot(x_plan, y_plan_eng, '-o', 'LineWidth', 2, 'MarkerSize', 8 , 'MarkerFaceColor','b');
hold on;
plot(x_plan, yhat_plan_eng, '--s', 'LineWidth', 1, 'MarkerSize', 4 , 'MarkerFaceColor','r');

xlabel('Duration'); ylabel('Energy Consumed');
title(sprintf('Sample %d: Phase = Planning',samplePlanning));
grid on

% plot travel prediction model

% Plot testing data

