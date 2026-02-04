% Trim Optimization Script for 4-Engine Aircraft
% Finds straight and level flight trim condition at 85 m/s
clear; clc; close all;

fprintf('=== Aircraft Trim Optimization (4 Engines) ===\n\n');

% Initialize Z_guess (9 states + 7 controls)
initialization = 0; % 0 = start from scratch, 1 = load previous trim

if initialization == 0
    fprintf('Starting optimization from initial guess...\n');
    Z_guess = zeros(16, 1);       % 9 states + 7 controls
    Z_guess(1) = 85;              % u: forward speed (m/s)
    Z_guess(3) = 0;               % w: vertical speed
    Z_guess(8) = 3*pi/180;        % theta: initial pitch angle guess
    Z_guess(10:16) = [0; 0; 0; 0.05; 0.05; 0.05; 0.05]; % controls
else
    fprintf('Loading previous trim solution...\n');
    load('trim_values_straight_level.mat', 'XStar', 'UStar')
    Z_guess = [XStar; UStar];
end

% Display initial cost
F_initial = cost_straight_level(Z_guess);
fprintf('Initial cost: %.6e\n\n', F_initial);

% Run optimization
fprintf('Running fminsearch optimization...\n');
options = optimset('TolX', 1e-10, 'MaxFunEvals', 10000, 'MaxIter', 10000, 'Display', 'iter');
[ZStar, F_final] = fminsearch(@cost_straight_level, Z_guess, options);

% Extract results
XStar = ZStar(1:9);
UStar = ZStar(10:16);

% Calculate trim parameters
XdotStar = TRIMRCAM(XStar, UStar);
VaStar = sqrt(XStar(1)^2 + XStar(2)^2 + XStar(3)^2);
alphaStar = atan2(XStar(3), XStar(1));
gammaStar = XStar(8) - alphaStar;

% Display results
fprintf('\n=== TRIM OPTIMIZATION RESULTS ===\n');
fprintf('Final cost: %.6e\n\n', F_final);

fprintf('--- Trim States ---\n');
fprintf('u (fwd vel):    %8.3f m/s\n', XStar(1));
fprintf('v (side vel):   %8.3f m/s\n', XStar(2));
fprintf('w (vert vel):   %8.3f m/s\n', XStar(3));
fprintf('p (roll rate):  %8.5f rad/s\n', XStar(4));
fprintf('q (pitch rate): %8.5f rad/s\n', XStar(5));
fprintf('r (yaw rate):   %8.5f rad/s\n', XStar(6));
fprintf('phi (roll):     %8.3f deg\n', XStar(7)*180/pi);
fprintf('theta (pitch):  %8.3f deg\n', XStar(8)*180/pi);
fprintf('psi (yaw):      %8.3f deg\n\n', XStar(9)*180/pi);

fprintf('--- Trim Controls ---\n');
fprintf('Aileron:    %8.3f deg\n', UStar(1)*180/pi);
fprintf('Stabilizer: %8.3f deg\n', UStar(2)*180/pi);
fprintf('Rudder:     %8.3f deg\n', UStar(3)*180/pi);
fprintf('Throttle 1: %8.3f deg\n', UStar(4)*180/pi);
fprintf('Throttle 2: %8.3f deg\n', UStar(5)*180/pi);
fprintf('Throttle 3: %8.3f deg\n', UStar(6)*180/pi);
fprintf('Throttle 4: %8.3f deg\n\n', UStar(7)*180/pi);

fprintf('--- Flight Parameters ---\n');
fprintf('Airspeed Va:      %8.3f m/s\n', VaStar);
fprintf('Angle of attack:  %8.3f deg\n', alphaStar*180/pi);
fprintf('Flight path angle: %8.5f deg\n', gammaStar*180/pi);

fprintf('\n--- State Derivatives (should be near zero) ---\n');
fprintf('Max |xdot|: %.6e\n', max(abs(XdotStar)));

% Save results
save('trim_values_straight_level.mat', 'XStar', 'UStar');
fprintf('\nTrim values saved to: trim_values_straight_level.mat\n');