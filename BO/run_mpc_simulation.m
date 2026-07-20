function cost = run_mpc_simulation(model)

    clear mbc_car_display;
    simIn = Simulink.SimulationInput(model);
    % simIn = simIn.setVariable("BO_params", BO_params);
    simIn = simIn.setModelParameter( ...
        "StopTime", "6.5", ...
        "ReturnWorkspaceOutputs", "on");
    
    simOut = sim(simIn);
    
    cost = BO_performance(simOut);

end