% System parameters
a1 = 0;       % example numbers, plug yours in
a2 = 0;
b1 = 1;

A = [0 1; a2 a1];
B = [0; b1];


% Desired arrived time rho
rho = 4;

% ========== POLE PLACEMENT METHOD (COMMENTED OUT) ==========
% % Desired poles
% imagine part , same pole 
% p = [-4/rho -4/rho];
% 
% % Compute feedback gains
% K_mat = place(A, B, p);
% K = -K_mat;   % match your control law: u = KX + ubar

% ========== LQR METHOD ==========
% LQR weight matrices - scaled with rho for desired settling time
Q = [10/4^2, 0;      % Position error penalty (scales with rho)
     0,        1/4];  % Velocity error penalty (scales with rho)
R = 1;                 % Control effort penalty

% Compute LQR feedback gains
[K_mat, S, E] = lqr(A, B, Q, R);
K = -K_mat;   % match your control law: u = KX + ubar

% Display eigenvalues for comparison
fprintf('LQR eigenvalues: %.3f, %.3f\n', real(E(1)), real(E(2)));
fprintf('Desired poles (from pole placement): %.3f, %.3f\n', -4/rho, -6/rho);

% Desired final state
y = 20;          % move to x = 1
XE = [y; 0];

% Compute ubar
k1 = K(2);
k2 = K(1);
ubar = -(a2 + b1*k2)/b1 * y;

% Simulation using ODE45
f = @(t,X) (A + B*K)*X + B*ubar;

tspan = [0 4];
X0 = [0; 0];

[t, X] = ode45(f, tspan, X0);

plot(t, X)
legend('x','xdot')
xlabel('Time (s)')
ylabel('State')
grid on