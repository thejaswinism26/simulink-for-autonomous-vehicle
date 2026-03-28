function y = P_N_dot_Angles(euler)
phi   = euler(1);
theta = euler(2);
psi   = euler(3);

y = [
    cos(theta)*cos(psi);
    cos(phi)*sin(psi);
    sin(phi)*sin(theta)*cos(psi);
    sin(phi)*sin(psi);
    cos(phi)*sin(theta)*cos(psi)
];
