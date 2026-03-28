% Plotting script for RCAM simulation results
% Run this AFTER running the Simulink simulation

% Check if simulation output exists
whos out

simX = out.simX;
simU = out.simU;
% assume 'out' is Simulink.SimulationOutput
% get simX from common locations
if isprop(out,'simX')
    simX = out.simX;
elseif isprop(out,'logsout') && ~isempty(out.logsout) && ~isempty(out.logsout.get('simX'))
    simX = out.logsout.get('simX'); % SimulationData.Signal
else
    error('simX not found in SimulationOutput');
end

% extract numeric time and data
if isa(simX,'timeseries')
    T = simX.Time;
    X = simX.Data;
elseif isa(simX,'Simulink.SimulationData.Signal') || isa(simX,'Simulink.SimulationData.SignalList')
    ts = simX.Values;       % often a timeseries or timeseries array
    T = ts.Time;
    X = ts.Data;
elseif isstruct(simX) && isfield(simX,'signals') && isfield(simX.signals,'values')
    T = simX.time;
    X = simX.signals.values;
else
    error('Unsupported simX type: %s', class(simX));
end

% plot (for multi-column X, plot each column)
plot(T, X)
xlabel('Time')
ylabel('Signal')
legendStrings = arrayfun(@(k) sprintf('col%d',k), 1:size(X,2), 'UniformOutput', false);
legend(legendStrings, 'Location','best')


%% Extract time and data from simulation output
t = simX.Time;  % Time vector
X = simX.Data;  % State data (9 columns)
U = simU.Data;  % Control data (7 columns)


%% Extract individual states
u_vel = X(:,1);    % Forward velocity (m/s)
v_vel = X(:,2);    % Side velocity (m/s)
w_vel = X(:,3);    % Vertical velocity (m/s)
p = X(:,4);        % Roll rate (rad/s)
q = X(:,5);        % Pitch rate (rad/s)
r = X(:,6);        % Yaw rate (rad/s)
phi = X(:,7);      % Roll angle (rad)
theta = X(:,8);    % Pitch angle (rad)
psi = X(:,9);      % Yaw angle (rad)

%% Extract control inputs
d_A = U(:,1);      % Aileron
d_T = U(:,2);      % Stabilizer
d_R = U(:,3);      % Rudder
d_th1 = U(:,4);    % Throttle 1
d_th2 = U(:,5);    % Throttle 2
d_th3 = U(:,6);    % Throttle 3
d_th4 = U(:,7);    % Throttle 4

%% Limit data to 60 seconds
idx = t <= 60;

t_plot = t(idx);

u_plot = u_vel(idx);
v_plot = v_vel(idx);
w_plot = w_vel(idx);

p_plot = p(idx)*180/pi;
q_plot = q(idx)*180/pi;
r_plot = r(idx)*180/pi;

phi_plot   = phi(idx)*180/pi;
theta_plot = theta(idx)*180/pi;
psi_plot   = psi(idx)*180/pi;

%% 3x3 Layout — Dark theme, NO grid lines
figure(1)

% -------- Row 1 --------
subplot(3,3,1)
plot(t_plot, u_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Velocity in X-Axis')
ylabel('u (m/s)')

subplot(3,3,2)
plot(t_plot, p_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Rotation About X-Axis')
ylabel('p (deg/s)')

subplot(3,3,3)
plot(t_plot, phi_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Roll')
ylabel('\phi (deg)')


% -------- Row 2 --------
subplot(3,3,4)
plot(t_plot, v_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Velocity in Y-Axis')
ylabel('v (m/s)')

subplot(3,3,5)
plot(t_plot, q_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Rotation About Y-Axis')
ylabel('q (deg/s)')

subplot(3,3,6)
plot(t_plot, theta_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Pitch')
ylabel('\theta (deg)')


% -------- Row 3 --------
subplot(3,3,7)
plot(t_plot, w_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Velocity in Z-Axis')
ylabel('w (m/s)')
xlabel('Time (s)')

subplot(3,3,8)
plot(t_plot, r_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Rotation About Z-Axis')
ylabel('r (deg/s)')
xlabel('Time (s)')

subplot(3,3,9)
plot(t_plot, psi_plot, 'LineWidth', 1.5)
grid off
box on
set(gca,'XGrid','off','YGrid','off')
title('Yaw')
ylabel('\psi (deg)')
xlabel('Time (s)')
%% Display summary
disp('=================================================');
disp('Plotting complete!');
disp('=================================================');
disp(['Simulation time: ' num2str(t(end)) ' seconds']);
disp(['Number of data points: ' num2str(length(t))]);
disp(['Time step (average): ' num2str(mean(diff(t))) ' seconds']);
disp('=================================================');