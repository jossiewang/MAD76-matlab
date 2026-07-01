function u_min = run_fmincon(state, N, spline, u0, lb, ub,dt)
% Diese Funktion läuft NUR im MATLAB-Interpreter (keine Code-Generierung!)
cost_function = @(u) mpc_cost(u, state, N, spline,dt);
%options = optimoptions('fmincon', 'Display', 'none', 'Algorithm', 'sqp');
[u_min, ~] = fmincon(cost_function, u0, [], [], [], [], lb, ub, []);
end