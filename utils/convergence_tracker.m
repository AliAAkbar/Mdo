function tracker = convergence_tracker(action, tracker, varargin)
% CONVERGENCE_TRACKER  Track and visualize optimization convergence.
%   tracker = convergence_tracker('init', [], n_obj)
%   tracker = convergence_tracker('update', tracker, gen, obj_values, constraints)
%   convergence_tracker('plot', tracker)
%   convergence_tracker('save', tracker, filename)
%
%   Actions:
%     'init'   - Initialize tracker struct
%     'update' - Add data for current generation
%     'plot'   - Generate convergence plots
%     'save'   - Save tracker to .mat file
%
%   Inputs:
%     action   - String: 'init', 'update', 'plot', or 'save'
%     tracker  - Tracker struct (pass [] for 'init')
%     varargin - Additional arguments depend on action
%
%   Output:
%     tracker  - Updated tracker struct

    switch lower(action)
        case 'init'
            tracker = init_tracker(varargin{:});
        case 'update'
            tracker = update_tracker(tracker, varargin{:});
        case 'plot'
            plot_convergence(tracker);
        case 'save'
            save_tracker(tracker, varargin{:});
        otherwise
            error('convergence_tracker: Unknown action "%s"', action);
    end

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function tracker = init_tracker(n_obj)
% INIT_TRACKER  Initialize convergence tracking struct.
    if nargin < 1; n_obj = 3; end

    tracker = struct();
    tracker.n_obj          = n_obj;
    tracker.generations    = [];
    tracker.best_fitness   = [];       % [n_gen x n_obj] Best per objective
    tracker.mean_fitness   = [];       % [n_gen x n_obj] Mean per objective
    tracker.worst_fitness  = [];       % [n_gen x n_obj] Worst per objective
    tracker.hypervolume    = [];       % [n_gen x 1] Hypervolume indicator
    tracker.n_feasible     = [];       % [n_gen x 1] Number of feasible individuals
    tracker.n_pareto       = [];       % [n_gen x 1] Pareto front size
    tracker.diversity      = [];       % [n_gen x 1] Population diversity metric
    tracker.constraint_viol = [];     % [n_gen x 1] Mean constraint violation
    tracker.timestamp      = [];       % [n_gen x 1] Elapsed time (seconds)
    tracker.start_time     = tic;
end

function tracker = update_tracker(tracker, gen, pop_objectives, constraints, hv)
% UPDATE_TRACKER  Record data for one generation.
%   tracker = update_tracker(tracker, gen, pop_objectives, constraints, hv)
%
%   Inputs:
%     gen            - Generation number
%     pop_objectives - [pop_size x n_obj] Objective values for entire population
%     constraints    - [pop_size x n_constraints] Constraint values (g <= 0 = feasible)
%     hv             - Scalar hypervolume value (optional, pass [] to skip)

    n_obj = tracker.n_obj;

    % Best, mean, worst for each objective
    best_obj = min(pop_objectives, [], 1);
    mean_obj = mean(pop_objectives, 1);
    worst_obj = max(pop_objectives, [], 1);

    % Feasibility
    if ~isempty(constraints)
        feasible_mask = all(constraints <= 0, 2);
        n_feasible = sum(feasible_mask);
        mean_viol = mean(max(0, constraints(:)));
    else
        n_feasible = size(pop_objectives, 1);
        mean_viol = 0;
    end

    % Population diversity (std of objectives, normalized)
    diversity = mean(std(pop_objectives, 0, 1) ./ max(range(pop_objectives, 1), 1e-10));

    % Elapsed time
    elapsed = toc(tracker.start_time);

    % Append data
    tracker.generations(end+1, 1)     = gen;
    tracker.best_fitness(end+1, :)    = best_obj;
    tracker.mean_fitness(end+1, :)    = mean_obj;
    tracker.worst_fitness(end+1, :)   = worst_obj;
    tracker.n_feasible(end+1, 1)      = n_feasible;
    tracker.diversity(end+1, 1)       = diversity;
    tracker.constraint_viol(end+1, 1) = mean_viol;
    tracker.timestamp(end+1, 1)       = elapsed;

    % Hypervolume
    if nargin >= 5 && ~isempty(hv)
        tracker.hypervolume(end+1, 1) = hv;
    else
        tracker.hypervolume(end+1, 1) = NaN;
    end

    % Pareto front size (approximate: n_feasible non-dominated)
    tracker.n_pareto(end+1, 1) = NaN;  % Updated externally if needed
end

function plot_convergence(tracker)
% PLOT_CONVERGENCE  Generate convergence visualization.
    n_gen = numel(tracker.generations);
    if n_gen < 2
        warning('convergence_tracker: Not enough data to plot.');
        return;
    end

    gen = tracker.generations;

    figure('Name', 'MDO Convergence', 'Position', [100, 100, 1200, 800]);

    % --- Subplot 1: Best fitness per objective ---
    subplot(2, 3, 1);
    plot(gen, tracker.best_fitness, 'LineWidth', 1.5);
    xlabel('Generation');
    ylabel('Best Objective Value');
    title('Best Fitness Evolution');
    legend({'CL/CD', 'Weight [kg]', 'Range [km]'}, 'Location', 'best');
    grid on;

    % --- Subplot 2: Mean fitness per objective ---
    subplot(2, 3, 2);
    plot(gen, tracker.mean_fitness, 'LineWidth', 1.5);
    xlabel('Generation');
    ylabel('Mean Objective Value');
    title('Mean Fitness Evolution');
    legend({'CL/CD', 'Weight [kg]', 'Range [km]'}, 'Location', 'best');
    grid on;

    % --- Subplot 3: Hypervolume ---
    subplot(2, 3, 3);
    hv = tracker.hypervolume;
    hv_valid = ~isnan(hv);
    if any(hv_valid)
        plot(gen(hv_valid), hv(hv_valid), 'b-', 'LineWidth', 2);
        xlabel('Generation');
        ylabel('Hypervolume');
        title('Hypervolume Indicator');
        grid on;
    else
        text(0.5, 0.5, 'No HV data', 'HorizontalAlignment', 'center');
    end

    % --- Subplot 4: Feasibility ---
    subplot(2, 3, 4);
    bar(gen, tracker.n_feasible, 'FaceColor', [0.3, 0.7, 0.3]);
    xlabel('Generation');
    ylabel('# Feasible');
    title('Feasible Individuals');
    grid on;

    % --- Subplot 5: Diversity ---
    subplot(2, 3, 5);
    plot(gen, tracker.diversity, 'r-', 'LineWidth', 1.5);
    xlabel('Generation');
    ylabel('Diversity Index');
    title('Population Diversity');
    grid on;

    % --- Subplot 6: Constraint Violation ---
    subplot(2, 3, 6);
    plot(gen, tracker.constraint_viol, 'm-', 'LineWidth', 1.5);
    xlabel('Generation');
    ylabel('Mean Violation');
    title('Constraint Violations');
    grid on;

    % Save figure
    saveas(gcf, fullfile('output', 'convergence_plot.png'));
    fprintf('Convergence plot saved to output/convergence_plot.png\n');
end

function save_tracker(tracker, filename)
% SAVE_TRACKER  Save tracker struct to .mat file.
    if nargin < 2 || isempty(filename)
        filename = fullfile('output', 'convergence_history.mat');
    end
    save(filename, 'tracker');
    fprintf('Convergence history saved to: %s\n', filename);
end
