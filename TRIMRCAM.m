function XDOT = TRIMRCAM(X,U)
% TRIMRCAM computes the state derivatives for the RCAM model (4 engines)
% ------------------ States ------------------
x1 = X(1); % u
x2 = X(2); % v
x3 = X(3); % w
x4 = X(4); % p
x5 = X(5); % q
x6 = X(6); % r
x7 = X(7); % phi
x8 = X(8); % theta
x9 = X(9); % psi

% ------------------ Controls ------------------
u1 = U(1); % aileron
u2 = U(2); % stabilizer
u3 = U(3); % rudder
u4 = U(4); % throttle1
u5 = U(5); % throttle2
u6 = U(6); % throttle3
u7 = U(7); % throttle4

% ------------------ Aircraft constants ------------------
m = 120000;        % mass (kg)
cbar = 6.6;        % mean aerodynamic chord (m)
lt = 24.8;         % tail arm (m)
S = 260;           % wing area (m^2)
St = 64;           % tail area (m^2)
Xcg = 0.23*cbar; Ycg = 0; Zcg = 0.10*cbar;
Xac = 0.12*cbar; Yac = 0; Zac = 0;

% Inertia matrix (kg*m^2)
Ib = m*[40.07 0 -2.0923;
        0 64 0;
        -2.0923 0 99.92];
invIb = inv(Ib);

% Engine positions (4 engines)
Xapt = [0 0 0 0];
Yapt = [-7.94 -2.5 2.5 7.94];  % left outer, left inner, right inner, right outer
Zapt = [-1.9 -1.9 -1.9 -1.9];

% Air properties
rho = 1.225; 
g = 9.81;

% ------------------ Aerodynamics ------------------
Va = sqrt(x1^2 + x2^2 + x3^2);

% Avoid division by zero
if Va < 0.1
    Va = 0.1;
end

alpha = atan2(x3, x1);
beta = asin(x2/Va);
Q = 0.5*rho*Va^2;

% Simple aerodynamic coefficients (placeholder - replace with actual model)
CL = 0.3 + 5.5*alpha;  % Lift coefficient
CD = 0.02 + 0.5*CL^2;  % Drag coefficient (parabolic drag polar)
CY = -1.6*beta + 0.24*u3;  % Side force

% Aerodynamic forces in stability frame
FA_s = [-CD*Q*S; 
         CY*Q*S; 
        -CL*Q*S];

% Rotate to body frame
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];
FA_b = C_bs*FA_s;

% Aerodynamic moments (simplified)
Cl = -0.13*beta + 0.011*u1 + 0.02*x4;  % Roll moment coefficient
Cm = -0.15*alpha - 1.5*x5 + 0.2*u2;     % Pitch moment coefficient
Cn = 0.05*beta - 0.032*x6 - 0.08*u3;    % Yaw moment coefficient

MA_b = [Cl; Cm; Cn] * Q * S * cbar;

% ------------------ Gravity ------------------
g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*cos(x7)];
Fg_b = m*g_b;

% ------------------ Engine forces (4 engines) ------------------
% Each throttle input controls one engine
Feng = [u4; u5; u6; u7]*m*g;  % Individual engine thrusts
FE_b = [sum(Feng); 0; 0];     % Total thrust in body x-direction

% Engine moments (due to offset from CG)
ME_b = zeros(3,1);
for i = 1:4
    r = [Xcg - Xapt(i); 
         Yapt(i) - Ycg; 
         Zapt(i) - Zcg];
    ME_b = ME_b + cross(r, [Feng(i); 0; 0]);
end

% ------------------ Total forces and moments ------------------
F_b = FA_b + Fg_b + FE_b;
M_b = MA_b + ME_b;

% ------------------ State derivatives ------------------
% Translational dynamics
uvw_dot = F_b/m - cross([x4; x5; x6], [x1; x2; x3]);

% Rotational dynamics
pqr_dot = invIb * (M_b - cross([x4; x5; x6], Ib*[x4; x5; x6]));

% Euler angle kinematics
H_phi = [1, sin(x7)*tan(x8), cos(x7)*tan(x8);
         0, cos(x7),        -sin(x7);
         0, sin(x7)/cos(x8), cos(x7)/cos(x8)];
phi_dot = H_phi * [x4; x5; x6];

% Assemble state derivative vector
XDOT = [uvw_dot; pqr_dot; phi_dot];

end