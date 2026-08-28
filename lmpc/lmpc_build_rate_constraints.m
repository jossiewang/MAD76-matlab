function [Adu, bdu] = ...
    lmpc_build_rate_constraints(D, d, params)

% N  = params.N;
N = 20;

duMax = repmat(params.du_max, N, 1);

Adu = [
     D;
    -D
];

bdu = [
    duMax - d;
    duMax + d
];

end