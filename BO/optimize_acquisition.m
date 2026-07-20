function x_best = optimize_acquisition(acq_fun, lb, ub, num_starts)
    % Input validation
    validateattributes(lb, {'numeric'}, {'vector'});
    validateattributes(ub, {'numeric'}, {'vector'});
    validateattributes(num_starts, {'numeric'}, {'scalar', 'positive'});
    
    % Ensure row vectors for bounds
    lb = lb(:)';
    ub = ub(:)';
    dim = length(lb);
    
    % Initialize with proper dimensions
    x_starts = zeros(num_starts, dim);
    for i = 1:num_starts
        x_starts(i,:) = lb + rand(1, dim) .* (ub - lb);
    end
    
    % Initialize storage
    x_candidates = zeros(num_starts, dim);
    ei_values = zeros(num_starts, 1);
    
    % Optimization options
    options = optimoptions('fmincon', 'Display', 'off', 'Algorithm', 'sqp');
    
    % Perform optimizations
    for i = 1:num_starts
        [x_candidates(i,:), neg_ei] = fmincon(@(x) -acq_fun(x), x_starts(i,:), ...
            [], [], [], [], lb, ub, [], options);
        ei_values(i) = -neg_ei;
    end
    
    % Select best result
    [~, best_idx] = max(ei_values);
    x_best = x_candidates(best_idx,:)'; % Return as column vector
end