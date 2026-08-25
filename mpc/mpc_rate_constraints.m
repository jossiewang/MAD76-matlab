function [A, b] = mpc_rate_constraints(N, u_last, params)

    nvar = 2*N;

    % 4 inequalities per step:
    %  dun <= dun_max
    % -dun <= dun_max
    %  ddelta <= ddelta_max
    % -ddelta <= ddelta_max
    
    A = zeros(4*N, nvar);
    b = zeros(4*N, 1);

    dun_max     = params.dun_max;
    ddelta_max  = params.ddeltan_max;

    row = 1;

    for i = 1:N
        idx_un = 2*i - 1;
        idx_de = 2*i;

        if i == 1
            % un_1 - u_last(1) <= dun_max
            A(row, idx_un) = 1;
            b(row) = dun_max + u_last(1);
            row = row + 1;

            % -(un_1 - u_last(1)) <= dun_max
            A(row, idx_un) = -1;
            b(row) = dun_max - u_last(1);
            row = row + 1;

            % delta_1 - u_last(2) <= ddelta_max
            A(row, idx_de) = 1;
            b(row) = ddelta_max + u_last(2);
            row = row + 1;

            % -(delta_1 - u_last(2)) <= ddelta_max
            A(row, idx_de) = -1;
            b(row) = ddelta_max - u_last(2);
            row = row + 1;
        else
            idx_un_prev = 2*(i-1) - 1;
            idx_de_prev = 2*(i-1);

            % un_i - un_{i-1} <= dun_max
            A(row, idx_un) = 1;
            A(row, idx_un_prev) = -1;
            b(row) = dun_max;
            row = row + 1;

            % -(un_i - un_{i-1}) <= dun_max
            A(row, idx_un) = -1;
            A(row, idx_un_prev) = 1;
            b(row) = dun_max;
            row = row + 1;

            % delta_i - delta_{i-1} <= ddelta_max
            A(row, idx_de) = 1;
            A(row, idx_de_prev) = -1;
            b(row) = ddelta_max;
            row = row + 1;

            % -(delta_i - delta_{i-1}) <= ddelta_max
            A(row, idx_de) = -1;
            A(row, idx_de_prev) = 1;
            b(row) = ddelta_max;
            row = row + 1;
        end
    end
end