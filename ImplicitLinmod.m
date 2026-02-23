function [E, Ap, Bp] = ImplicitLinmod(model, xdot0, x0, u0, dxdot_mat, dx_mat, du_mat)
% ImplicitLinmod
% Linearizes an implicit system F(xdot,x,u)=0 around an operating point
%
% Inputs:
%   model      - function handle: F = model(xdot,x,u)
%   xdot0      - equilibrium state derivative
%   x0         - equilibrium state
%   u0         - equilibrium input
%   dxdot_mat  - structure/sparsity matrix for dF/dxdot
%   dx_mat     - structure/sparsity matrix for dF/dx
%   du_mat     - structure/sparsity matrix for dF/du
%
% Outputs:
%   E   - dF/dxdot
%   Ap  - dF/dx
%   Bp  - dF/du

nx = length(x0);
nu = length(u0);

E  = zeros(nx,nx);
Ap = zeros(nx,nx);
Bp = zeros(nx,nu);

% Nominal function value
F0 = model(xdot0, x0, u0);

%% dF / d(xdot)
for i = 1:nx
    if any(dxdot_mat(:,i) ~= 0)
        dxd = zeros(nx,1);
        dxd(i) = dxdot_mat(i,i);
        F1 = model(xdot0 + dxd, x0, u0);
        E(:,i) = (F1 - F0)/eps;
    end
end

%% dF / d(x)
for i = 1:nx
    if any(dx_mat(:,i) ~= 0)
        dx = zeros(nx,1);
        dx(i) = eps;
        dx(i)  = dx_mat(i,i);
        F1 = model(xdot0, x0 + dx, u0);
        Ap(:,i) = (F1 - F0)/eps;
    end
end

%% dF / d(u)
for i = 1:nu
    if any(du_mat(:,i) ~= 0)
        du = zeros(nu,1);
        du(i)  = du_mat(i,i);
        F1 = model(xdot0, x0, u0 + du);
        Bp(:,i) = (F1 - F0)/eps;
    end
end
end
