function J = BO_performance(simOut)

    % for this very simple test, the performance of one run depends only on
    % the integrated lateral deviation
    errorData = simOut.logsout.get("RMSE").Values;
    J = errorData.Data(end);

end