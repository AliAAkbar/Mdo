function [child1, child2] = crossover(parent1, parent2, eta, prob, lb, ub)
% CROSSOVER  Simulated Binary Crossover (SBX) operator.
%   [child1, child2] = crossover(parent1, parent2, eta, prob, lb, ub)
%
%   Inputs:
%     parent1 - [1 x n_vars] First parent
%     parent2 - [1 x n_vars] Second parent
%     eta     - Distribution index (higher = children closer to parents)
%     prob    - Crossover probability
%     lb      - [1 x n_vars] Lower bounds
%     ub      - [1 x n_vars] Upper bounds
%
%   Outputs:
%     child1  - [1 x n_vars] First offspring
%     child2  - [1 x n_vars] Second offspring

    n_vars = numel(parent1);
    child1 = parent1;
    child2 = parent2;

    if rand() > prob
        return;  % No crossover, children = parents
    end

    for j = 1:n_vars
        if rand() > 0.5
            continue;  % Variable-wise crossover probability
        end

        if abs(parent1(j) - parent2(j)) < 1e-14
            continue;  % Parents identical at this gene
        end

        % Ensure p1 < p2
        p1 = min(parent1(j), parent2(j));
        p2 = max(parent1(j), parent2(j));

        % Compute beta
        diff = p2 - p1;
        beta1 = 1 + 2 * (p1 - lb(j)) / diff;
        beta2 = 1 + 2 * (ub(j) - p2) / diff;

        % Compute alpha
        alpha1 = 2 - beta1^(-(eta + 1));
        alpha2 = 2 - beta2^(-(eta + 1));

        % Generate betaq for child 1
        u1 = rand();
        if u1 <= 1/alpha1
            betaq1 = (u1 * alpha1)^(1/(eta + 1));
        else
            betaq1 = (1/(2 - u1 * alpha1))^(1/(eta + 1));
        end

        % Generate betaq for child 2
        u2 = rand();
        if u2 <= 1/alpha2
            betaq2 = (u2 * alpha2)^(1/(eta + 1));
        else
            betaq2 = (1/(2 - u2 * alpha2))^(1/(eta + 1));
        end

        % Compute children
        child1(j) = 0.5 * ((p1 + p2) - betaq1 * diff);
        child2(j) = 0.5 * ((p1 + p2) + betaq2 * diff);

        % Enforce bounds
        child1(j) = max(lb(j), min(ub(j), child1(j)));
        child2(j) = max(lb(j), min(ub(j), child2(j)));
    end

end
