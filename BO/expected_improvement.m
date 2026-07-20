function ei = expected_improvement(x, gpModel, Y)
    % Ensure x is properly formatted for 3D input
    if isvector(x)
        x = reshape(x, 1, length(x));
    end
    
    % Predict using GP model
    [mu, sigma] = predict(gpModel, x);
    
    % Calculate EI
    y_best = min(Y);
    z = (y_best - mu) ./ max(sigma, eps);
    ei = (y_best - mu) .* normcdf(z) + sigma .* normpdf(z);
    ei(sigma < eps) = 0;
end