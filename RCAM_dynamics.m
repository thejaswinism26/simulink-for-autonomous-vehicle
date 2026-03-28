function xdot = RCAM_dynamics(x, u)
    % RCAM_dynamics - Wrapper for RCAM_mod to match expected interface
    %
    % Inputs:
    %   x - [9x1] state vector [u v w p q r phi theta psi]
    %   u - [7x1] control vector [ail stab rud thr1 thr2 thr3 thr4]
    %
    % Output:
    %   xdot - [9x1] state derivatives
    
    % Call the existing RCAM_mod function
    xdot = RCAM_mod(x, u);
end