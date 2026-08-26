function [useq_k, iter] = linear_MPC_quadprog(x_k, qp, info, useq_k_0)
%LINEAR_MPC_QUADPROG Solves the QP problem that is equivalent to the
%considered MPC problem for the given initial state.
%   x_k:        Initial state
%               (n by 1) matrix
%   qp:         Describes the QP that is equivalent to the MPC problem.
%               Structure with fields:
%               - H     (N*m by N*m) matrix
%               - L     (n by N*m) matrix
%               - ub    (N*m by 1) matrix
%               - G     (2*(N+1)*n by N*m) matrix
%               - S     (2*(N+1)*n by n) matrix
%               - w     (2*(N+1)*n by 1) matrix
%   info:       Determines whether a solver output is printed or not.
%               Boolean
%   useq_k_0:   Control input sequence of length N. Can be neglected.
%               (N*m by 1) matrix.
%               If it is IS provided, then it is used as the initial guess
%               for solving the MPC problem via an active set solver.
%               If it is NOT provided, then the MPC problem is solved via
%               an interior point solver.
%   useq_k:     Optimal control input sequence for the given initial state.
%   iter:       Number of iterations required by the solver.
%               Positive real number

% evaluate current linear cost term
f_k = qp.L' * x_k;

% evaluate current polyhedral constraints
w_k = qp.w - qp.S * x_k;

% choose solver
if nargin == 3 % NO initial guess was provided -> use interior point solver
    options = optimoptions('quadprog', ...
        'Algorithm', 'interior-point-convex');
    % no initial guess required for interior point methods
    useq_k_0 = [];
else % initial guess WAS provided -> use active set solver
    % set active set solver
    options = optimoptions('quadprog', ...
        'Algorithm', 'active-set');
end
% set display option
if info
    options.Display = 'final';
else
    options.Display = 'off';
end

% Solve the QP
[useq_k, ~, flag, output] = quadprog(qp.H, ...  % quadratic objective term
    f_k, ...                                    % linear objective term
    qp.G, ...                                   % lhs of polyhedral constraints
    w_k, ...                                    % rhs of polyhedral constraints
    [], ...                                     % lhs of equality constraints
    [], ...                                     % rhs of equality constraints
    -qp.ub, ...                                 % lower bound on u_seq
    qp.ub, ...                                  % upper bound on u_seq
    useq_k_0, ...                               % initial guess
    options ...                                 % solver options
    );

% check exit flag
if flag ~= 1
    error('quadprog did not converge to a solution!');
else
    % extract number of iterations
    iter = output.iterations;
end
end