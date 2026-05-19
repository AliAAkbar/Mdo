function results = results_struct(action, varargin)
% RESULTS_STRUCT  Build, save, and display structured optimization results.
%   results = results_struct('build', opt_results, tracker, prob_cfg, moo_cfg)
%   results_struct('save', results, filename)
%   results_struct('display', results)
%
%   Actions:
%     'build'   - Assemble comprehensive results struct
%     'save'    - Save to .mat and generate summary report
%     'display' - Print formatted summary to console
%
%   Output:
%     results - Comprehensive results struct

    switch lower(action)
        case 'build'
            results = build_results(varargin{:});
        case 'save'
            save_results(varargin{:});
            results = [];
        case 'display'
            display_results(varargin{:});
            results = [];
        otherwise
            error('results_struct: Unknown action "%s"', action);
    end

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function results = build_results(opt_results, tracker, prob_cfg, moo_cfg)
% BUILD_RESULTS  Assemble comprehensive output struct.
    results = struct();

    % --- Metadata ---
    results.metadata.timestamp      = datestr(now);
    results.metadata.mode           = prob_cfg.mode;
    results.metadata.n_vars         = prob_cfg.n_vars;
    results.metadata.var_names      = prob_cfg.var_names;
    results.metadata.n_objectives   = moo_cfg.n_objectives;
    results.metadata.objective_names = moo_cfg.objective_names;
    results.metadata.matlab_version = version;

    % --- Optimization settings used ---
    results.settings.prob_cfg = prob_cfg;
    results.settings.moo_cfg  = moo_cfg;

    % --- Results (mode-dependent) ---
    if strcmpi(prob_cfg.mode, 'multi')
        results.pareto.front          = opt_results.pareto_front;
        results.pareto.set            = opt_results.pareto_set;
        results.pareto.n_solutions    = size(opt_results.pareto_front, 1);

        % Knee point
        knee_idx = pareto_front('filter', opt_results.pareto_front);
        results.pareto.knee_point     = opt_results.pareto_front(knee_idx, :);
        results.pareto.knee_design    = opt_results.pareto_set(knee_idx, :);
        results.pareto.knee_index     = knee_idx;

        % Best per objective
        for k = 1:moo_cfg.n_objectives
            sense = moo_cfg.obj_sense(k);
            if sense == -1  % Maximize
                [~, idx] = max(opt_results.pareto_front(:, k));
            else            % Minimize
                [~, idx] = min(opt_results.pareto_front(:, k));
            end
            results.pareto.best_per_obj(k).objective_value = opt_results.pareto_front(idx, k);
            results.pareto.best_per_obj(k).design_vector   = opt_results.pareto_set(idx, :);
            results.pareto.best_per_obj(k).index           = idx;
        end
    else
        % Single objective
        results.optimum.design_vector = opt_results.best_individual;
        results.optimum.fitness       = opt_results.best_fitness;
        results.optimum.converged     = opt_results.converged;
    end

    % --- Convergence data ---
    results.convergence.generations_run  = opt_results.generations_run;
    results.convergence.tracker          = tracker;
    if isfield(tracker, 'hypervolume')
        valid_hv = tracker.hypervolume(~isnan(tracker.hypervolume));
        if ~isempty(valid_hv)
            results.convergence.final_hypervolume = valid_hv(end);
        end
    end

    % --- Final population ---
    results.population.individuals = opt_results.final_population;
    if isfield(opt_results, 'final_objectives')
        results.population.objectives = opt_results.final_objectives;
    end

end

function save_results(results, filename)
% SAVE_RESULTS  Save results to .mat file and generate text report.
    if nargin < 2 || isempty(filename)
        filename = fullfile('output', 'mdo_results.mat');
    end

    % Ensure output directory exists
    [fpath, ~, ~] = fileparts(filename);
    if ~isempty(fpath) && ~exist(fpath, 'dir')
        mkdir(fpath);
    end

    % Save .mat
    save(filename, 'results', '-v7.3');
    fprintf('Results saved to: %s\n', filename);

    % Generate text report
    report_file = strrep(filename, '.mat', '_report.txt');
    fid = fopen(report_file, 'w');
    if fid == -1
        warning('Cannot write report file.');
        return;
    end

    fprintf(fid, '================================================================\n');
    fprintf(fid, '  MDO OPTIMIZATION RESULTS - Electric Amphibious Aircraft\n');
    fprintf(fid, '================================================================\n');
    fprintf(fid, 'Timestamp: %s\n', results.metadata.timestamp);
    fprintf(fid, 'Mode: %s\n', results.metadata.mode);
    fprintf(fid, 'Generations run: %d\n', results.convergence.generations_run);
    fprintf(fid, 'Number of design variables: %d\n', results.metadata.n_vars);
    fprintf(fid, '\n');

    if strcmpi(results.metadata.mode, 'multi')
        fprintf(fid, '--- PARETO FRONT ---\n');
        fprintf(fid, 'Number of Pareto solutions: %d\n', results.pareto.n_solutions);
        fprintf(fid, '\nKnee Point (compromise solution):\n');
        obj_names = results.metadata.objective_names;
        for k = 1:numel(obj_names)
            fprintf(fid, '  %s: %.4f\n', obj_names{k}, results.pareto.knee_point(k));
        end
        fprintf(fid, '\nKnee Point Design Variables:\n');
        var_names = results.metadata.var_names;
        for k = 1:numel(var_names)
            fprintf(fid, '  %s: %.4f\n', var_names{k}, results.pareto.knee_design(k));
        end
        fprintf(fid, '\n--- BEST PER OBJECTIVE ---\n');
        for k = 1:numel(obj_names)
            fprintf(fid, '%s: %.4f\n', obj_names{k}, ...
                    results.pareto.best_per_obj(k).objective_value);
        end
    else
        fprintf(fid, '--- OPTIMAL SOLUTION ---\n');
        fprintf(fid, 'Best fitness: %.6f\n', results.optimum.fitness);
        fprintf(fid, 'Converged: %s\n', mat2str(results.optimum.converged));
        fprintf(fid, '\nDesign Variables:\n');
        var_names = results.metadata.var_names;
        for k = 1:numel(var_names)
            fprintf(fid, '  %s: %.4f\n', var_names{k}, results.optimum.design_vector(k));
        end
    end

    fprintf(fid, '\n================================================================\n');
    fclose(fid);
    fprintf('Report saved to: %s\n', report_file);
end

function display_results(results)
% DISPLAY_RESULTS  Print formatted summary to console.
    fprintf('\n');
    fprintf('╔══════════════════════════════════════════════════════════════╗\n');
    fprintf('║    MDO RESULTS - Electric Amphibious Aircraft               ║\n');
    fprintf('╠══════════════════════════════════════════════════════════════╣\n');
    fprintf('║ Mode: %-54s ║\n', results.metadata.mode);
    fprintf('║ Generations: %-47d ║\n', results.convergence.generations_run);

    if strcmpi(results.metadata.mode, 'multi')
        fprintf('║ Pareto solutions: %-42d ║\n', results.pareto.n_solutions);
        fprintf('╠══════════════════════════════════════════════════════════════╣\n');
        fprintf('║ KNEE POINT (Compromise Solution):                           ║\n');
        obj_names = results.metadata.objective_names;
        for k = 1:numel(obj_names)
            fprintf('║   %-15s: %10.4f                                ║\n', ...
                    obj_names{k}, results.pareto.knee_point(k));
        end
    else
        fprintf('╠══════════════════════════════════════════════════════════════╣\n');
        fprintf('║ OPTIMAL FITNESS: %.6f                                   ║\n', ...
                results.optimum.fitness);
    end

    fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');
end
