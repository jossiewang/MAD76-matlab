function J = BO_performance(simOut)

    % for this very simple test, the performance of one run depends only on
    % the integrated lateral deviation
    errorData = simOut.logsout.get("lapTime").Values;
    feasibilityData = simOut.logsout.get("runSuccessful").Values;
    J = errorData.Data(end) + 6*(1-feasibilityData.Data(end));

end