function [XDOT] = RCAM_mod(X,U)

%% ================== STATE VECTOR ==================
x1 = X(1); % u
x2 = X(2); % v
x3 = X(3); % w
x4 = X(4); % p
x5 = X(5); % q
x6 = X(6); % r
x7 = X(7); % phi
x8 = X(8); % theta
x9 = X(9); % psi

%% ================== CONTROL VECTOR (7 inputs) ==================
u1 = U(1); % aileron
u2 = U(2); % stabilizer
u3 = U(3); % rudder
u4 = U(4); % throttle 1
u5 = U(5); % throttle 2
u6 = U(6); % throttle 3
u7 = U(7); % throttle 4

%% ================== AIRCRAFT CONSTANTS ==================
m    = 7.3;          
cbar = 0.316992;
lt   = 0.9499975;
S    = 0.75395152;
St   = 0.12497767;

Xcg = 0.28702;
Ycg = 0.127;
Zcg = 1.188974;

Xac = 0.3566795;
Yac = 0.127;
Zac = 1.188974;

% Engine positions
Xapt1 = 0; Yapt1 = -0.865632; Zapt1 = -0.066802;
Xapt2 = 0; Yapt2 = -2.5;      Zapt2 = -0.066802;
Xapt3 = 0; Yapt3 = 0.407416;  Zapt3 = -0.066802;
Xapt4 = 0; Yapt4 = 0.865632;  Zapt4 = -0.066802;

rho = 1.225;
g   = 9.81;

depsda      = 0.25;
alpha_L0    = -11.5*pi/180;
n           = 5.5;
a3 = -768.5; 
a2 = 609.2; 
a1 = -155.2; 
a0 = 15.212;
alpha_switch = 14.5*pi/180;

%% ================== CONTROL SATURATION ==================
u1 = min(max(u1,-25*pi/180),  25*pi/180);
u2 = min(max(u2,-25*pi/180),  10*pi/180);
u3 = min(max(u3,-30*pi/180),  30*pi/180);
u4 = min(max(u4, 0.5*pi/180), 10*pi/180);
u5 = min(max(u5, 0.5*pi/180), 10*pi/180);
u6 = min(max(u6, 0.5*pi/180), 10*pi/180);
u7 = min(max(u7, 0.5*pi/180), 10*pi/180);

%% ================== INTERMEDIATE VARIABLES ==================
Va = sqrt(x1^2 + x2^2 + x3^2);
Va = max(Va,0.1);     % Prevent division by zero

alpha = atan2(x3,x1);
beta  = asin(x2/Va);

Q = 0.5*rho*Va^2;

wbe_b = [x4;x5;x6];
V_b   = [x1;x2;x3];

%% ================== AERODYNAMIC COEFFICIENTS ==================
if alpha <= alpha_switch
    CL_wb = n*(alpha - alpha_L0);
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0;
end

epsilon = depsda*(alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
CL_t    = 3.1*(St/S)*alpha_t;

CL = CL_wb + CL_t;
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2;
CY = -1.6*beta + 0.24*u3;

%% ================== AERODYNAMIC FORCES ==================
FA_s = [-CD*Q*S;
         CY*Q*S;
        -CL*Q*S];

C_bs = [ cos(alpha) 0 -sin(alpha);
         0          1  0;
         sin(alpha) 0  cos(alpha)];

FA_b = C_bs * FA_s;

%% ================== AERODYNAMIC MOMENTS ==================
eta = [ -1.4*beta;
        -0.59 - (3.1*(St*lt)/(S*cbar))*(alpha-epsilon);
        (1 - alpha*(180/(15*pi)))*beta ];

dCMdx = (cbar/Va)*[-11 0 5;
                   0 (-4.03*(St*lt^2)/(S*cbar^2)) 0;
                   1.7 0 -11.5];

dCMdu = [-0.6 0 0.22;
          0 (-3.1*(St*lt)/(S*cbar)) 0;
          0 0 -0.63];

CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];
MAac_b = CMac_b * Q * S * cbar;

rcg_b = [Xcg;Ycg;Zcg];
rac_b = [Xac;Yac;Zac];

MAcg_b = MAac_b + cross(FA_b, rcg_b - rac_b);

%% ================== ENGINE FORCES & MOMENTS ==================
F1 = u4*m*g;
F2 = u5*m*g;
F3 = u6*m*g;
F4 = u7*m*g;

FE1_b = [F1;0;0];
FE2_b = [F2;0;0];
FE3_b = [F3;0;0];
FE4_b = [F4;0;0];

FE_b = FE1_b + FE2_b + FE3_b + FE4_b;

mew1 = [Xcg-Xapt1; Yapt1-Ycg; Zcg-Zapt1];
mew2 = [Xcg-Xapt2; Yapt2-Ycg; Zcg-Zapt2];
mew3 = [Xcg-Xapt3; Yapt3-Ycg; Zcg-Zapt3];
mew4 = [Xcg-Xapt4; Yapt4-Ycg; Zcg-Zapt4];

MEcg_b = cross(mew1,FE1_b) + ...
         cross(mew2,FE2_b) + ...
         cross(mew3,FE3_b) + ...
         cross(mew4,FE4_b);

%% ================== GRAVITY ==================
g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*cos(x7)];

Fg_b = m*g_b;

%% ================== DYNAMICS ==================
Ib = m*[40.07 0 -2.0923;
        0 64 0;
       -2.0923 0 99.92];

invIb = (1/m)*[0.0249836 0 0.000523151;
               0 0.015625 0;
               0.000523151 0 0.010019];

% Translational
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b,V_b);

% Rotational
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b,Ib*wbe_b));

% Euler angle kinematics
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7)        -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];

x7to9dot = H_phi*wbe_b;

%% ================== STATE DERIVATIVE ==================
XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot];

end
