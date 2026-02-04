function F0 = cost_straight_level(Z)
% Cost function for straight and level flight trim (4 engines)
% Z = [X(1:9); U(1:7)] where X is state, U is control

X = Z(1:9);     % Extract states
U = Z(10:16);   % Extract controls (3 surfaces + 4 throttles)

% Calculate state derivatives
xdot = TRIMRCAM(X, U);

% Calculate flight parameters
Va = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
alpha = atan2(X(3), X(1));
gamma = X(8) - alpha;  % Flight path angle

% Define trim requirements (all should be zero at trim)
Q = [xdot;        % All 9 state derivatives should be zero
     Va - 85;     % Target airspeed: 85 m/s
     gamma;       % Flight path angle = 0 (level)
     X(2);        % Lateral velocity v = 0 (no sideslip)
     X(7);        % Roll angle phi = 0 (wings level)
     X(9)];       % Yaw angle psi = 0 (or any constant heading)

% Cost function (sum of squared errors)
H = eye(length(Q));  % Identity weighting matrix
F0 = Q'*H*Q;

end