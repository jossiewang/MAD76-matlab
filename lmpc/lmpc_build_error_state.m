function x0 = ...
    lmpc_build_error_state(states, refs)

%% Velocity error

ve = states(1) - refs.v(1);


%% Position error

pos_err = states(2:3) - refs.s(:,1);

sc1e = refs.t(:,1).' * pos_err;
sc2e = refs.n(:,1).' * pos_err;


%% Heading error

dpsi = states(4) - refs.psi(1);

psie = atan2( ...
    sin(dpsi), ...
    cos(dpsi));


%% Error state

x0 = [
    ve;
    sc1e;
    sc2e;
    psie
    ];

end