function cost = run_mpc_simulation(model)

pause(5);
clear mbc_car_display;
simIn = Simulink.SimulationInput(model);
% simIn = simIn.setVariable("BO_params", BO_params);
simIn = simIn.setModelParameter( ...
    "StopTime", "15", ...
    "ReturnWorkspaceOutputs", "on");
% simIn = simIn.setModelParameter("ReturnWorkspaceOutputs", "on");
simOut = sim(simIn);

cost = BO_performance(simOut);

end