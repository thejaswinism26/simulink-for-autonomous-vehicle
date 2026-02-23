simulinkModelName = "navcomb.slx";

% Aircraft parameters
m = 120000; % Aircraft total mass (kg)

cbar = 6.6;     % Mean Aerodynamic Chord (m)
lt = 24.8;      % Distance by AC of tail and body (m)
S = 260;        % Wing planform area (m^2)
St = 64;        % Tail planform area (m^2)
Xcg = 0.23*cbar; % x position of CoG in Fm (m)
Ycg = 0;         % y position of CoG in Fm (m)
Zcg = 0.10*cbar; % z position of CoG in Fm (m)
Xac = 0.12*cbar; % x position of aerodynamic center in Fm (m) 
Yac = 0;         % y position of aerodynamic center in Fm (m)
Zac = 0;         % z position of aerodynamic center in Fm (m)

% Engine constants - NOW 4 ENGINES
Xapt1 = 0;       % x position of engine 1 force in Fm (m)
Yapt1 = -7.94;   % y position of engine 1 force in Fm (m) - left outer
Zapt1 = -1.9;    % z position of engine 1 force in Fm (m)

Xapt2 = 0;       % x position of engine 2 force in Fm (m)
Yapt2 = -2.5;    % y position of engine 2 force in Fm (m) - left inner
Zapt2 = -1.9;    % z position of engine 2 force in Fm (m)

Xapt3 = 0;       % x position of engine 3 force in Fm (m)
Yapt3 = 2.5;     % y position of engine 3 force in Fm (m) - right inner
Zapt3 = -1.9;    % z position of engine 3 force in Fm (m)

Xapt4 = 0;       % x position of engine 4 force in Fm (m)
Yapt4 = 7.94;    % y position of engine 4 force in Fm (m) - right outer
Zapt4 = -1.9;    % z position of engine 4 force in Fm (m)

% Other constants
rho = 1.225;     % Air density (kg/m^3)
g = 9.81;        % Gravity (m/s^2)
depsda = 0.25;
alpha_L0 = -11.5*pi/180;
n = 5.5;
a3 = -768.5;
a2 = 609.2;
a1 = -155.2;
a0 = 15.212;
alpha_switch = 14.5*(pi/180);

% Inertia matrix
Ib = m*[40.07 0 -2.0923;
        0 64 0;
        -2.0923 0 99.92];

% Inverse inertia matrix (optional - for reference)
invIb = (1/m)*[0.0249836 0 0.000523151;
               0 0.015625 0;
               0.000523151 0 0.010019];

% ========================================================================
% CORRECTED: Initial state vector with 12 states (geodetic navigation)
% ========================================================================
% State order: [PN, PE, PD, u, v, w, phi, theta, psi, p, q, r]
x0 = [0;     % 1. PN - North position (m)
      0;     % 2. PE - East position (m)
      -1000; % 3. PD - Down position (m) [negative = altitude of 1000m]
      100;   % 4. u - forward velocity (m/s)
      0;     % 5. v - side velocity (m/s)
      0;     % 6. w - vertical velocity (m/s)
      0;     % 7. phi - roll angle (rad)
      0;     % 8. theta - pitch angle (rad)
      0;     % 9. psi - yaw angle (rad)
      0;     % 10. p - roll rate (rad/s)
      0;     % 11. q - pitch rate (rad/s)
      0];    % 12. r - yaw rate (rad/s)

% Initial control inputs - 7 CONTROLS (3 surfaces + 4 throttles)
u0 = [0;     % d_A - aileron (rad)
      0;     % d_T - stabilizer (rad)
      0;     % d_R - rudder (rad)
      0.05;  % d_th1 - throttle 1 (fraction of weight)
      0.05;  % d_th2 - throttle 2 (fraction of weight)
      0.05;  % d_th3 - throttle 3 (fraction of weight)
      0.05]; % d_th4 - throttle 4 (fraction of weight)

u = u0;
x = x0;

% Simulation time parameters
TF = 100;         % Final simulation time (s)
dt = 0.01;        % Time step (s)

% Display confirmation
disp('=================================================');
disp('Variables initialized successfully');
disp('=================================================');
disp(['Aircraft mass: ' num2str(m) ' kg']);
disp(['Wing area: ' num2str(S) ' m^2']);
disp(['Initial altitude: ' num2str(-x0(3)) ' m']);
disp(['Initial forward velocity: ' num2str(x0(4)) ' m/s']);
disp(['Number of states: ' num2str(length(x0))]);
disp(['Number of engines: 4']);
disp(['Number of control inputs: ' num2str(length(u0)) ' (3 surfaces + 4 throttles)']);
disp('=================================================');
disp(' ');
disp('Initial State Vector (12x1):');
disp(['  PN (North):     ' num2str(x0(1)) ' m']);
disp(['  PE (East):      ' num2str(x0(2)) ' m']);
disp(['  PD (Down):      ' num2str(x0(3)) ' m (Alt = ' num2str(-x0(3)) ' m)']);
disp(['  u (velocity):   ' num2str(x0(4)) ' m/s']);
disp(['  v (velocity):   ' num2str(x0(5)) ' m/s']);
disp(['  w (velocity):   ' num2str(x0(6)) ' m/s']);
disp(['  phi (roll):     ' num2str(x0(7)*180/pi) ' deg']);
disp(['  theta (pitch):  ' num2str(x0(8)*180/pi) ' deg']);
disp(['  psi (yaw):      ' num2str(x0(9)*180/pi) ' deg']);
disp(['  p (roll rate):  ' num2str(x0(10)) ' rad/s']);
disp(['  q (pitch rate): ' num2str(x0(11)) ' rad/s']);
disp(['  r (yaw rate):   ' num2str(x0(12)) ' rad/s']);
disp('=================================================');
disp(' ');
disp('Next steps:');
disp('1. Open your Simulink model: open_system(''navcomb.slx'')');
disp('2. Make sure Demux 2 is set to 12 outputs');
disp('3. Run the simulation');
disp('=================================================');