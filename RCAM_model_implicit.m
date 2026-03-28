function F = RCAM_model_implicit(xdot, x, u)
    % RCAM_model_implicit - Implicit formulation of RCAM aircraft dynamics
    % F(xdot, x, u) = 0
    %
    % Inputs:
    %   xdot - [9x1] state derivatives
    %   x    - [9x1] state vector [u v w p q r phi theta psi]
    %   u    - [7x1] control vector [ail stab rud thr1 thr2 thr3 thr4]
    %
    % Output:
    %   F    - [9x1] residual vector (should be zero at equilibrium)
    
    % Call your RCAM dynamics function
    f = RCAM_dynamics(x, u);
    
    % Implicit formulation: F = xdot - f(x,u) = 0
    F = xdot - f;
end