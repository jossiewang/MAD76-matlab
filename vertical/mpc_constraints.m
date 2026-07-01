
% MPC constraints function
function [c, ceq] = mpc_constraints(u, x0, mpc_params, sys_params)
    % Initialize state
    x=x0;

    % Initialize constraints
    c = []; % Inequality constraints c(u) <= 0
    ceq = []; % Equality constraints ceq(u) == 0

    % Unpack MPC parameters
    xmin = mpc_params.x_min;
    xmax = mpc_params.x_max;
    %theta_min=mpc_params.theta_min;
    %theta_max=mpc_params.theta_max;
    dt=mpc_params.dt;
    apply_constraints=mpc_params.apply_constraints;

    N= mpc_params.N;
    % Loop over prediction horizon
    for i=1:N
        x=simulate_dynamics(x,u(i),dt,sys_params); %ggf vor if
        if apply_constraints==1
            c=[c;
            x(1)-xmax;  % x(1) ist position
            xmin-x(1);
            %x(3)-theta_max;
            %theta_min-x(3); %x(3) ist theta, hier gibt es keine constr
            ];
            
        end
    end
end