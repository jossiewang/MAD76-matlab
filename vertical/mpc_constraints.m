% MPC constraints function
function [c, ceq] = mpc_constraints(u, state, N, spline,dt,x_min,x_max)
    % Initialize state
    x = state;

    % Initialize constraints
    c = []; % Inequality constraints c(u) <= 0
    ceq = []; % Equality constraints ceq(u) == 0

    theta_max = 1.2;
    theta_min = -theta_max;
    apply_constraints=1;
    % Loop over prediction horizon
    for k = 1:N
        % Apply control input
        u_k = u(k:k+1);

        % Simulate dynamics using RK4
        x = simulate_dynamics_rk4(x, u_k, dt);
        [x_diff,~]=error_dynamics(spline,x);

        if apply_constraints
            % Append inequality constraints for state limits
            % State constraints: x_min <= x <= x_max
            c = [c;
                 double(x_diff(1) - x_max);        % x <= x_max -> x(1) - x_max <= 0
                 double(x_min - x_diff(1));        % x >= x_min -> x_min - x(1) <= 0
                 double(x(2) - theta_max);    % theta <= theta_max -> x(3) - theta_max <= 0
                 double(theta_min - x(2));   % theta >= theta_min -> theta_min - x(3) <= 0
                 ];
        end
    end
end