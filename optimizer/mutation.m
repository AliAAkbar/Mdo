function individual = mutation(individual, eta, prob, lb, ub)
% MUTATION  Polynomial mutation operator.
%   individual = mutation(individual, eta, prob, lb, ub)
%
%   Inputs:
%     individual - [1 x n_vars] Individual to mutate
%     eta        - Distribution index (higher = smaller perturbation)
%     prob       - Mutation probability per variable
%     lb         - [1 x n_vars] Lower bounds
%     ub         - [1 x n_vars] Upper bounds
%
%   Output:
%     individual - [1 x n_vars] Mutated individual

    n_vars = numel(individual);

    for j = 1:n_vars
        if rand() > prob
            continue;
        end

        x = individual(j);
        delta_max = ub(j) - lb(j);

        if delta_max < 1e-14
            continue;
        end

        % Normalized distances
        delta1 = (x - lb(j)) / delta_max;
        delta2 = (ub(j) - x) / delta_max;

        u = rand();

        if u < 0.5
            % Left side
            xy = 1 - delta1;
            val = 2*u + (1 - 2*u) * xy^(eta + 1);
            deltaq = val^(1/(eta + 1)) - 1;
        else
            % Right side
            xy = 1 - delta2;
            val = 2*(1 - u) + 2*(u - 0.5) * xy^(eta + 1);
            deltaq = 1 - val^(1/(eta + 1));
        end

        % Apply mutation
        individual(j) = x + deltaq * delta_max;

        % Enforce bounds
        individual(j) = max(lb(j), min(ub(j), individual(j)));
    end

end
