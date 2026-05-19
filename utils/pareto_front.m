function pf = pareto_front(action, varargin)
% PARETO_FRONT  Extract, filter, and visualize Pareto front.
%   pf = pareto_front('extract', obj_values)
%   pareto_front('plot', pf_obj, pf_vars, obj_names)
%   pareto_front('save', pf_obj, pf_vars, filename)
%
%   Actions:
%     'extract' - Find non-dominated solutions from objective matrix
%     'plot'    - Generate Pareto front visualization (2D/3D)
%     'save'    - Export Pareto front data
%     'filter'  - Apply knee-point or preference selection
%
%   Output:
%     pf - Depends on action:
%          'extract' → Struct with .objectives, .indices, .n_points
%          'filter'  → Index of selected knee point

    switch lower(action)
        case 'extract'
            pf = extract_pareto(varargin{:});
        case 'plot'
            plot_pareto(varargin{:});
            pf = [];
        case 'save'
            save_pareto(varargin{:});
            pf = [];
        case 'filter'
            pf = filter_pareto(varargin{:});
        otherwise
            error('pareto_front: Unknown action "%s"', action);
    end

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function pf = extract_pareto(obj_values)
% EXTRACT_PARETO  Find non-dominated solutions.
%   Assumes all objectives are to be MINIMIZED.
%   Apply obj_sense before calling this function.

    N = size(obj_values, 1);
    is_dominated = false(N, 1);

    for i = 1:N
        if is_dominated(i)
            continue;
        end
        for j = i+1:N
            if is_dominated(j)
                continue;
            end
            if dominates_pf(obj_values(i,:), obj_values(j,:))
                is_dominated(j) = true;
            elseif dominates_pf(obj_values(j,:), obj_values(i,:))
                is_dominated(i) = true;
                break;
            end
        end
    end

    pf_indices = find(~is_dominated);

    pf = struct();
    pf.objectives = obj_values(pf_indices, :);
    pf.indices = pf_indices;
    pf.n_points = numel(pf_indices);
end

function result = dominates_pf(a, b)
% DOMINATES_PF  Returns true if a dominates b (all minimized).
    result = all(a <= b) && any(a < b);
end

function plot_pareto(pf_obj, pf_vars, obj_names, var_names)
% PLOT_PARETO  Visualize Pareto front.
    if nargin < 3; obj_names = {'Obj 1', 'Obj 2', 'Obj 3'}; end
    if nargin < 4; var_names = {}; end

    [n_points, n_obj] = size(pf_obj);

    figure('Name', 'Pareto Front', 'Position', [150, 150, 1400, 600]);

    if n_obj == 2
        % --- 2D Pareto Front ---
        subplot(1, 2, 1);
        scatter(pf_obj(:,1), pf_obj(:,2), 50, 'b', 'filled');
        xlabel(obj_names{1});
        ylabel(obj_names{2});
        title('Pareto Front');
        grid on;

    elseif n_obj >= 3
        % --- 3D Pareto Front ---
        subplot(1, 2, 1);
        scatter3(pf_obj(:,1), pf_obj(:,2), pf_obj(:,3), 50, ...
                 pf_obj(:,3), 'filled');
        xlabel(obj_names{1});
        ylabel(obj_names{2});
        zlabel(obj_names{3});
        title('3D Pareto Front');
        colorbar;
        colormap(jet);
        grid on;
        view(45, 30);

        % --- 2D projections ---
        subplot(2, 2, 2);
        scatter(pf_obj(:,1), pf_obj(:,2), 30, pf_obj(:,3), 'filled');
        xlabel(obj_names{1});
        ylabel(obj_names{2});
        title([obj_names{1}, ' vs ', obj_names{2}]);
        colorbar;
        grid on;

        subplot(2, 2, 4);
        scatter(pf_obj(:,1), pf_obj(:,3), 30, pf_obj(:,2), 'filled');
        xlabel(obj_names{1});
        ylabel(obj_names{3});
        title([obj_names{1}, ' vs ', obj_names{3}]);
        colorbar;
        grid on;
    end

    % Save
    saveas(gcf, fullfile('output', 'pareto_front.png'));
    fprintf('Pareto front plot saved to output/pareto_front.png\n');

    % --- Parallel coordinates of Pareto set (design variables) ---
    if ~isempty(pf_vars) && ~isempty(var_names)
        figure('Name', 'Pareto Set - Design Variables', 'Position', [200, 200, 1000, 400]);

        % Normalize variables to [0, 1] for visualization
        pf_norm = (pf_vars - min(pf_vars, [], 1)) ./ ...
                  max(range(pf_vars, 1), 1e-10);

        plot(1:size(pf_norm, 2), pf_norm', 'Color', [0.2, 0.4, 0.8, 0.3]);
        set(gca, 'XTick', 1:numel(var_names), 'XTickLabel', var_names);
        xtickangle(45);
        ylabel('Normalized Value');
        title('Pareto Set - Parallel Coordinates');
        grid on;

        saveas(gcf, fullfile('output', 'pareto_set_parallel.png'));
    end
end

function save_pareto(pf_obj, pf_vars, filename)
% SAVE_PARETO  Export Pareto front data.
    if nargin < 3 || isempty(filename)
        filename = fullfile('output', 'pareto_front_data.mat');
    end

    pareto_data = struct();
    pareto_data.objectives = pf_obj;
    pareto_data.design_variables = pf_vars;
    pareto_data.n_solutions = size(pf_obj, 1);
    pareto_data.timestamp = datestr(now);

    save(filename, 'pareto_data');
    fprintf('Pareto front data saved to: %s\n', filename);

    % Also save as CSV for external tools
    csv_file = strrep(filename, '.mat', '.csv');
    combined = [pf_obj, pf_vars];
    % Write header
    fid = fopen(csv_file, 'w');
    fprintf(fid, 'CL_CD,Weight_kg,Range_km');
    for i = 1:size(pf_vars, 2)
        fprintf(fid, ',Var%d', i);
    end
    fprintf(fid, '\n');
    fclose(fid);
    % Append data
    dlmwrite(csv_file, combined, '-append', 'delimiter', ',', 'precision', 6);
    fprintf('Pareto front CSV saved to: %s\n', csv_file);
end

function knee_idx = filter_pareto(pf_obj)
% FILTER_PARETO  Find knee point of Pareto front.
%   Uses maximum distance from the utopia-nadir line.

    n_points = size(pf_obj, 1);
    if n_points <= 1
        knee_idx = 1;
        return;
    end

    % Normalize objectives to [0, 1]
    obj_min = min(pf_obj, [], 1);
    obj_max = max(pf_obj, [], 1);
    obj_range = obj_max - obj_min;
    obj_range(obj_range < 1e-10) = 1;

    pf_norm = (pf_obj - obj_min) ./ obj_range;

    % Knee point: maximize minimum distance to extremes
    % Simple approach: point closest to ideal (all zeros after normalization)
    dist_to_ideal = sqrt(sum(pf_norm.^2, 2));
    [~, knee_idx] = min(dist_to_ideal);
end
