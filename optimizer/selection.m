function idx = selection(fitness, tournament_size, n_select)
% SELECTION  Tournament selection for genetic algorithm.
%   idx = selection(fitness, tournament_size, n_select)
%
%   Inputs:
%     fitness         - [N x 1] Fitness values (lower is better)
%     tournament_size - Number of individuals per tournament
%     n_select        - Number of individuals to select
%
%   Output:
%     idx - [n_select x 1] Indices of selected individuals

    pop_size = numel(fitness);
    idx = zeros(n_select, 1);

    for i = 1:n_select
        % Randomly pick tournament_size individuals
        candidates = randi(pop_size, tournament_size, 1);
        % Select the one with best (lowest) fitness
        [~, best] = min(fitness(candidates));
        idx(i) = candidates(best);
    end

end
