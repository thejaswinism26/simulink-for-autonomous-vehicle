clear
clc
close all

%% ==================== TRIM CONDITION SETUP ====================
fprintf('Setting up trim condition...\n');

% State derivatives at trim (equilibrium)
Xdoto = zeros(9,1);  % All state derivatives = 0 at trim

% Trim state vector
Xo = [
    84.9905    % u - forward velocity (m/s)
    0          % v - side velocity (m/s)
    1.2713     % w - vertical velocity (m/s)
    0          % p - roll rate (rad/s)
    0          % q - pitch rate (rad/s)
    0          % r - yaw rate (rad/s)
    0          % phi - roll angle (rad)
    0.0150     % theta - pitch angle (rad)
    0          % psi - yaw angle (rad)
];

% Trim control vector (7 controls for 4 engines)
Uo = [
    0          % aileron (rad)
    -0.1780    % stabilizer/elevator (rad)
    0          % rudder (rad)
    0.0821     % throttle 1 (0-1 normalized)
    0.0821     % throttle 2 (0-1 normalized)
    0.0821     % throttle 3 (0-1 normalized)
    0.0821     % throttle 4 (0-1 normalized)
];

fprintf('Trim state set:\n');
fprintf('  Velocity: u=%.2f m/s, v=%.2f m/s, w=%.2f m/s\n', Xo(1), Xo(2), Xo(3));
fprintf('  Pitch angle: theta=%.4f rad (%.2f deg)\n', Xo(8), Xo(8)*180/pi);
fprintf('  Elevator: %.4f rad (%.2f deg)\n', Uo(2), Uo(2)*180/pi);
fprintf('  Throttles: [%.4f %.4f %.4f %.4f]\n\n', Uo(4), Uo(5), Uo(6), Uo(7));

%% ==================== VERIFY TRIM EQUILIBRIUM ====================
fprintf('Verifying trim condition...\n');

% Check if trim is actually an equilibrium
F_trim = RCAM_model_implicit(Xdoto, Xo, Uo);
F_trim_norm = norm(F_trim);

fprintf('Equilibrium check: ||F(xdot,x,u)|| = %.6e\n', F_trim_norm);
if F_trim_norm < 1e-6
    fprintf('✓ Trim condition is a valid equilibrium\n\n');
else
    fprintf('⚠ WARNING: Trim may not be at equilibrium (should be < 1e-6)\n');
    fprintf('   You may need to adjust Xo and Uo values\n\n');
end

%% ==================== PERTURBATION MATRICES ====================
fprintf('Setting up perturbation matrices...\n');

% Perturbation size
epsilon = 1e-8;

% Perturbation matrices (diagonal)
dxdot_matrix = epsilon * eye(9);   % For state derivatives (9x9)
dx_matrix = epsilon * eye(9);      % For states (9x9)
du_matrix = epsilon * eye(7);      % For controls (7x7) - CORRECTED!

fprintf('Perturbation size: %.2e\n\n', epsilon);

%% ==================== PERFORM LINEARIZATION ====================
fprintf('========================================\n');
fprintf('Starting implicit linearization...\n');
fprintf('========================================\n');

% Call ImplicitLinmod to compute E, Ap, Bp matrices
[E, Ap, Bp] = ImplicitLinmod(@RCAM_model_implicit, Xdoto, Xo, Uo, ...
                              dxdot_matrix, dx_matrix, du_matrix);

fprintf('\n========================================\n');
fprintf('Linearization complete!\n');
fprintf('========================================\n\n');

%% ==================== COMPUTE STATE-SPACE MATRICES ====================
fprintf('Computing state-space matrices A and B...\n');

% Check if E is invertible
cond_E = cond(E);
fprintf('Condition number of E: %.2e\n', cond_E);

if cond_E > 1e12
    warning('E matrix is poorly conditioned! Results may be inaccurate.');
end

% Convert from implicit form E*xdot = Ap*x + Bp*u
% to standard form xdot = A*x + B*u
A = -E\Ap;  % Using backslash is more numerically stable than inv(E)
B = -E\Bp;

fprintf('✓ A and B matrices computed\n\n');

%% ==================== DISPLAY RESULTS ====================
fprintf('========================================\n');
fprintf('LINEARIZED STATE-SPACE MODEL\n');
fprintf('========================================\n\n');

disp('=== STATE MATRIX A (9x9) ===')
disp('Describes how states evolve naturally (system dynamics)')
disp('States: [u v w p q r phi theta psi]')
A

disp('=== INPUT MATRIX B (9x7) ===')
disp('Describes how control inputs affect state derivatives')
disp('Inputs: [aileron stabilizer rudder thr1 thr2 thr3 thr4]')
B

%% ==================== STABILITY ANALYSIS ====================
fprintf('========================================\n');
fprintf('STABILITY ANALYSIS\n');
fprintf('========================================\n');

eigenvalues = eig(A);

fprintf('\nEigenvalues of A matrix:\n');
fprintf('%-20s %-20s %-20s\n', 'Real Part', 'Imaginary Part', 'Magnitude');
fprintf('%-20s %-20s %-20s\n', '----------', '--------------', '---------');
for i = 1:length(eigenvalues)
    fprintf('%-20.6f %-20.6f %-20.6f\n', ...
            real(eigenvalues(i)), imag(eigenvalues(i)), abs(eigenvalues(i)));
end

fprintf('\n');
num_unstable = sum(real(eigenvalues) > 0);
num_stable = sum(real(eigenvalues) < 0);
num_marginally = sum(abs(real(eigenvalues)) < 1e-6);

fprintf('Summary:\n');
fprintf('  Stable modes (Re < 0):     %d\n', num_stable);
fprintf('  Unstable modes (Re > 0):   %d\n', num_unstable);
fprintf('  Marginal modes (Re ≈ 0):   %d\n\n', num_marginally);

if all(real(eigenvalues) < 0)
    fprintf('✓ SYSTEM IS STABLE at this trim point\n');
elseif any(real(eigenvalues) > 0)
    fprintf('✗ SYSTEM IS UNSTABLE at this trim point\n');
    fprintf('  Number of unstable modes: %d\n', num_unstable);
else
    fprintf('⚠ SYSTEM IS MARGINALLY STABLE at this trim point\n');
end

%% ==================== SAVE RESULTS ====================
fprintf('\n========================================\n');
fprintf('SAVING RESULTS\n');
fprintf('========================================\n');

save('linearized_model_4engine.mat', 'A', 'B', 'E', 'Ap', 'Bp', 'Xo', 'Uo', 'eigenvalues');

fprintf('✓ Linearized model saved to: linearized_model_4engine.mat\n');
fprintf('  Variables saved: A, B, E, Ap, Bp, Xo, Uo, eigenvalues\n');

fprintf('\n========================================\n');
fprintf('LINEARIZATION COMPLETE!\n');
fprintf('========================================\n');