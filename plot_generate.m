% plot_generate.m
% Demonstrate R_i(t) and J_i(t) for unmonitored target
% R_i(t) = R_i(t_o) + A_i(t - t_o)
% J_i(t) = R_i(t_o)(t-t_o) + (1/2)*A_i*(t-t_o)^2

clear; clc; close all;

% Parameters
t_o = 0;
t_f = 10;
R_i_t0 = 5;    % R_i(t_o)
A_i = 1.5;     % growth rate

% Time vector
t = linspace(t_o, t_f, 200);

% Equations
R_i = R_i_t0 + A_i * (t - t_o);
J_i = R_i_t0 * (t - t_o) + 0.5 * A_i * (t - t_o).^2;

% Plot
figure;

subplot(2, 1, 1);
plot(t, R_i, 'b-', 'LineWidth', 2);
hold on; xline(6, 'k--', 'LineWidth', 1); hold off;
ylabel('R_i(t)');
xlim([t_o t_f]);
grid on;
legend('R_i(t)', 'Location', 'northwest');
title('Uncertainty');

subplot(2, 1, 2);
plot(t, J_i, 'r-', 'LineWidth', 2);
hold on; xline(6, 'k--', 'LineWidth', 1); hold off;
ylabel('J_i(t)');
xlabel('Time t');
xlim([t_o t_f]);
grid on;
legend('J_i(t)', 'Location', 'northwest');
title('Objective');

linkaxes(findobj(gcf, 'Type', 'axes'), 'x');  % sync time axis