function [U_sim,X_sim, V_0] = dp_forward_simulation(sys,N,K_matrix,S_matrix,xbar)
%DP_FORWARD_SIMULATION Realizes the second step of the Dynamic Programming
%algorithm: it computes the input and state trajectories for the initial
%state x_0 = xbar and the specified time-varying feedback law K_k
%   sys:        Describes the discrete-time LTI system.
%               Structure with fields:
%               - A     (n by n) matrix
%               - B     (n by m) matrix
%   weights:    Describes the cost of the LQR problem
%               Structure with fields:
%               - Q     (n by n) matrix
%               - R     (m by m) matrix
%               - P     (n by n) matrix
%   N:          Horizon length
%               Natural number.
%   K_matrix:   Specifies the time-varying feedback law
%               (m by n by N) matrix. 
%               The entry (:, :, k+1) contains the feedback gain at time
%               k, for k=0,1,...,N-1 
%   S_matrix:   Specifies the time-varying value function
%               (n by n by N+1) matrix. 
%               The entry (:, :, k+1) contains the Riccati matrix S_k at
%               time k, for k=0,1,...,N 
%   xbar:       Initial state
%               (n by 1) matrix
%   U_sim:      Optimal input sequence for the given initial state.
%               (m by N) matrix.
%               The entry (:, k+1) contains the optimal input at time k,
%               for k=0,1,...,N-1
%   X_sim:      Optimal state sequence for the given initial state.
%               (n by N+1) matrix.
%               The entry (:, k+1) contains the optimal state at time k,
%               for k=0,1,...,N. 
%               The entry (:, 1) contains the initial state.
%   V_0:        Optimal value for the LQR problem.
%               Real number.

% system dimensions
[n,m] = size(sys.B);

% pre-allocate...
X_sim = NaN(n,N+1);
U_sim = NaN(m,N);
% .. and initialize
X_sim(:,1) = xbar;

% loop over the horizon
for index=1:N % k = index - 1, we use 'index' for easier indexing in Matlab
    % current state & feedback gain
    x_k = X_sim(:,index);
    K_k = K_matrix(:,:,index);

    % evaluate controller
    u_k = K_k * x_k;

    % store input...
    U_sim(:,index) = u_k;
    % ... and apply it
    X_sim(:,index+1) = sys.A * x_k + sys.B * u_k;
end
% optimal value over the horizon
S_0 = S_matrix(:,:,1);
V_0 = xbar' * (S_0 * xbar);
end