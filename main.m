%% MAIN.M - MDO of Electric Amphibious Aircraft
%  =========================================================================
%  Multidisciplinary Design Optimization (MDO) framework for an electric
%  amphibious aircraft using OpenVSP for aerodynamics and MATLAB for all
%  other disciplines (propulsion, structures, hydrodynamics, optimization).
%
%  Supports:
%    - Single-objective GA (weighted aggregate)
%    - Multi-objective GA (NSGA-II) with Pareto front generation
%
%  Usage:
%    >> main          % Run with default config (multi-objective)
%
%  Author: [Thesis Student]
%  Date:   2024
%  =========================================================================

%% Housekeeping
clear; clc; close all;
fprintf('================================================================\n');
fprintf('  MDO - Electric Amphibious Aircraft Optimization\n');
fprintf('  Framework v1.0\n');
fprintf('================================================================\n\n');

%% Add all module paths
addpath(genpath('config'));
addpath(genpath('optimizer'));
addpath(genpath('aero'));
addpath(genpath('propulsion'));
addpath(genpath('amphibious'));
addpath(genpath('objectives'));
addpath(genpath('utils'));

% Ensure output directory exists
if ~exist('output', 'dir')
    mkdir('output');
end

%% Load configurations
fprintf('[1/5] Loading configurations...\n');

prob_cfg  = problem_config();
ga_cfg    = ga_config();
moo_cfg   = moo_config();
prop_cfg  = propulsion_config();
amphi_cfg = amphibious_config();

fprintf('       Design variables: %d\n', prob_cfg.n_vars);
fprintf('       Optimization mode: %s\n', prob_cfg.mode);
fprintf('       Population size: %d\n', ga_cfg.pop_size);

%% Set up objective function wrapper
fprintf('[2/5] Setting up objective function...\n');

% Wrapper function that the optimizer calls for each individual.
% Captures all configs via closure.
if strcmpi(prob_cfg.mode, 'multi')
    % Multi-objective: returns [1 x n_obj] vector
    obj_func = @(x) moo_objective_wrapper(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
    fprintf('       Objectives: %d (%s)\n', moo_cfg.n_objectives, ...
            strjoin(moo_cfg.objective_names, ', '));
else
    % Single-objective: returns scalar (penalized weighted sum)
    obj_func = @(x) single_objective_wrapper(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);
    fprintf('       Mode: Single-objective (weighted sum)\n');
end

%% Initialize convergence tracker
fprintf('[3/5] Initializing tracker...\n');
tracker = convergence_tracker('init', [], moo_cfg.n_objectives);

%% Run optimization
fprintf('[4/5] Starting optimization...\n');
fprintf('       RNG seed: %d (reproducible)\n', ga_cfg.rng_seed);
fprintf('       ---\n');

tic;

if strcmpi(prob_cfg.mode, 'multi')
    % ===== MULTI-OBJECTIVE (NSGA-II) =====
    opt_results = ga_multi(obj_func, prob_cfg, ga_cfg, moo_cfg);
else
    % ===== SINGLE-OBJECTIVE GA =====
    opt_results = ga_single(obj_func, prob_cfg, ga_cfg);
end

elapsed_time = toc;
fprintf('\n       Optimization completed in %.1f seconds.\n', elapsed_time);
fprintf('       Generations run: %d\n', opt_results.generations_run);

%% Post-processing
fprintf('[5/5] Post-processing results...\n');

% Build comprehensive results struct
results = results_struct('build', opt_results, tracker, prob_cfg, moo_cfg);
results.metadata.elapsed_time = elapsed_time;

% Display summary
results_struct('display', results);

% Save results
results_struct('save', results, fullfile('output', 'mdo_results.mat'));

% Generate convergence plots (if sufficient data)
if isfield(opt_results, 'history') && ~isempty(opt_results.history)
    % For multi-objective, plot Pareto front
    if strcmpi(prob_cfg.mode, 'multi')
        pareto_front('plot', opt_results.pareto_front, opt_results.pareto_set, ...
                     moo_cfg.objective_names, prob_cfg.var_names);
        pareto_front('save', opt_results.pareto_front, opt_results.pareto_set, ...
                     fullfile('output', 'pareto_front_data.mat'));
    end
end

% Save convergence history
convergence_tracker('save', tracker, fullfile('output', 'convergence_history.mat'));

fprintf('\n================================================================\n');
fprintf('  Optimization complete. Results saved to output/\n');
fprintf('================================================================\n');

%% ========================================================================
%  OBJECTIVE FUNCTION WRAPPERS (local functions)
%  ========================================================================

function f = moo_objective_wrapper(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg)
% MOO_OBJECTIVE_WRAPPER  Multi-objective function wrapper.
%   Returns [1 x n_obj] raw objective values.
%   Constraint handling via penalty or feasibility-first is done in ga_multi.

    try
        [obj_values, g, ~] = evaluate_objectives(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);

        % Apply penalty for infeasible solutions
        if strcmpi(moo_cfg.constraint_handling, 'penalty')
            violation = sum(max(0, g));
            if violation > 0
                penalty = moo_cfg.penalty_factor * violation;
                % Worsen all objectives by penalty
                obj_sense = moo_cfg.obj_sense;
                for k = 1:numel(obj_values)
                    if obj_sense(k) == -1  % Maximize → make smaller
                        obj_values(k) = obj_values(k) - penalty;
                    else                    % Minimize → make larger
                        obj_values(k) = obj_values(k) + penalty;
                    end
                end
            end
        end

        f = obj_values;

    catch ME
        % If evaluation fails, return worst-case objectives
        warning('Evaluation failed: %s', ME.message);
        f = zeros(1, moo_cfg.n_objectives);
        % Worst case for each objective direction
        for k = 1:moo_cfg.n_objectives
            if moo_cfg.obj_sense(k) == -1  % Maximize → return 0
                f(k) = 0;
            else  % Minimize → return large value
                f(k) = 1e6;
            end
        end
    end
end

function f = single_objective_wrapper(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg)
% SINGLE_OBJECTIVE_WRAPPER  Weighted-sum single-objective function.
%   Returns scalar fitness value (minimize).

    try
        [obj_values, g, ~] = evaluate_objectives(x, prob_cfg, prop_cfg, amphi_cfg, moo_cfg);

        % Weighted sum (normalize objectives first)
        % Weights: CL/CD (0.4), Weight (0.3), Range (0.3)
        weights = [0.4, 0.3, 0.3];

        % Normalize to approximate [0, 1] range
        norm_factors = [20, 1500, 200];  % Typical max values

        f = 0;
        for k = 1:numel(obj_values)
            normalized = obj_values(k) / norm_factors(k);
            if moo_cfg.obj_sense(k) == -1  % Maximize → negate for minimization
                f = f - weights(k) * normalized;
            else  % Minimize
                f = f + weights(k) * normalized;
            end
        end

        % Constraint penalty
        violation = sum(max(0, g));
        if violation > 0
            f = f + 1000 * violation;
        end

    catch ME
        warning('Single-obj evaluation failed: %s', ME.message);
        f = 1e6;  % Worst case
    end
end
