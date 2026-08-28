function J = BO_performance(simOut)

% Performance metric for one hardware/LQR run.
%
% A run is successful only if:
%   1. |Sc2e| never exceeds 0.06 m
%   2. lapDone becomes true at least once
%
% Cost:
%   J = final RMSE + penalty for unsuccessful run

%% Get logged signals
errorData = simOut.logsout.get("RMSE").Values;
Sc2eData  = simOut.logsout.get("Sc2e").Values;
lapData   = simOut.logsout.get("lapDone").Values;

%% Performance
rmse = errorData.Data(end);

%% Success checks

% 1. Lateral error must stay within +/- 0.06 m
trackOK = all(abs(Sc2eData.Data(:)) <= 0.06);

% 2. A complete lap must have been detected at least once
lapCompleted = any(logical(lapData.Data(:)));

% Overall success
runSuccessful = trackOK && lapCompleted;

%% BO objective
J = rmse + 0.02 * (1 - double(runSuccessful));

end