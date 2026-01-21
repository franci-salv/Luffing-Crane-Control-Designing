%% System Parameters and Initial Conditions

% Generalized coordinates vector: q = [x; theta]
% x: position of the hoist block (m)
% theta: absolute angle of the chain (rad)

% Initial generalized position q(0)
x0 = 12.5; % Initial hoist position (m)
theta0 = pi/2; % Initial chain angle (rad)
q0 = [x0; theta0]; 

% Initial generalized velocity q_dot(0)
x_dot0 = 0; % Initial hoist velocity (m/s)
theta_dot0 = 0; % Initial chain angular velocity (rad/s)
q_dot0 = [x_dot0; theta_dot0]; 

% Input signals (as specified)
%FA = 1500*sin(1.1096*time); % Force applied to the hoist (N) - constant
spd = 0; % Speed parameter (for any internal speed dependencies, if applicable)

% Simulation settings
T_sim = 130; % Simulation time in seconds

% Store initial conditions for Simulink (often used as separate variables)
% Note: In Simulink, the initial conditions are typically set in the 
% Integrator blocks directly, referencing these workspace variables.
initial_position = q0;
initial_velocity = q_dot0;


%% Run Simulink Model

% IMPORTANT: Replace 'your_simulink_model_name' with the actual name of your .slx file.
% Your Simulink model must:
% 1. Implement the EoMs: solve for q_ddot (q_double_dot) from M*q_double_dot = F.
% 2. Use two Integrator blocks in series to go from q_double_dot -> q_dot -> q.
% 3. Have 'To Workspace' blocks named 'simout_x' and 'simout_theta' 
%    (or similar) for the x(t) and theta(t) signals.
% 4. Use the variables defined above (e.g., FA, spd) as inputs or parameters.

% Extract state trajectories from q
addpath("C:\Users\franc\OneDrive\Bureaublad\codespace\Githubcode\college\Dyn and Cont\deliverable 1");
simOut = sim("TimeTraj_Del3.slx");
q_data = simOut.q;    % This is an array: [time x theta] or [x theta] depending on your block

% If q is saved as an array with columns [x, theta]:
x_data = q_data(:,1);
theta_data = q_data(:,2);

% Time vector
time = simOut.tout;




% Extract time and position data from the simulation output
% Assuming your 'To Workspace' blocks save data as 'timeseries' objects
% If you used a different format, adjust the extraction accordingly.

%% Load measured lab data

%% Load measured lab data


%% 📈 Plot 3: Hoist Position x(t) – Simulation vs Measured
figure(3);
plot(time, x_data, 'LineWidth', 1.5); hold on;

xlim([0 130]);
ylim([11.5 22.5]);

title('Hoist Position x(t): Simulation vs');
xlabel('Time (s)');
ylabel('Hoist Position x (m)');

legend('Simulation x(t)', 'Linear Simulation x(t)', 'Location', 'best');

grid on;


%% 📉 Plot 4: Chain Angle θ(t) – Simulation vs Measured
figure(4);
plot(time, theta_data, 'LineWidth', 1.5); hold on;

xlim([0 130]);
ylim([1.38 1.821]);

title('Chain Angle \theta(t): Simulation');
xlabel('Time (s)');
ylabel('Chain Angle \theta (rad)');

legend('Simulation \theta(t)', 'Linear Simulated \theta(t)', 'Location', 'best');

grid on;
