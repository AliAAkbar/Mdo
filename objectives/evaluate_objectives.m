function [obj_values, constraint_values, discipline_data] = evaluate_objectives(design_vars, prob_cfg, prop_cfg, amphi_cfg, moo_cfg)
% EVALUATE_OBJECTIVES  Master evaluation function for all objectives and constraints.
%   [obj_values, constraint_values, discipline_data] = evaluate_objectives(...)
%
%   This is the function called by the optimizer. It orchestrates all
%   discipline analyses and aggregates the results into objective and
%   constraint vectors.
%
%   Inputs:
%     design_vars - [1 x n_vars] Design variable vector
%     prob_cfg    - Problem config struct
%     prop_cfg    - Propulsion config struct
%     amphi_cfg   - Amphibious config struct
%     moo_cfg     - Multi-objective config struct
%
%   Outputs:
%     obj_values        - [1 x n_obj] Objective values (raw, before sense conversion)
%                         [CL/CD, Weight, Range]
%     constraint_values - [1 x n_constraints] Constraint violations (g <= 0 = feasible)
%     discipline_data   - Struct with all discipline results for post-processing
%
%   Objective directions (handled by optimizer via moo_cfg.obj_sense):
%     1. CL/CD  → maximize (obj_sense = -1)
%     2. Weight → minimize (obj_sense = +1)
%     3. Range  → maximize (obj_sense = -1)

    % =====================================================================
    % DISCIPLINE 1: AERODYNAMICS
    % =====================================================================
    aero_results = compute_aero(design_vars, prob_cfg, amphi_cfg);

    % =====================================================================
    % DISCIPLINE 2: PROPULSION
    % =====================================================================
    prop_results = propulsion_analysis(design_vars, prob_cfg, prop_cfg);

    % =====================================================================
    % DISCIPLINE 3: AMPHIBIOUS
    % =====================================================================
    amphi_results = amphibious_analysis(design_vars, prob_cfg, prop_cfg, ...
                                         amphi_cfg, aero_results, prop_results);

    % =====================================================================
    % OBJECTIVE 1: Aerodynamic Efficiency (CL/CD)
    % =====================================================================
    f1 = objective_clcd(aero_results);

    % =====================================================================
    % OBJECTIVE 2: Total Aircraft Weight
    % =====================================================================
    f2 = objective_weight(design_vars, prob_cfg, prop_results, amphi_results);

    % =====================================================================
    % OBJECTIVE 3: Range
    % =====================================================================
    f3 = objective_range(aero_results, prop_results, prob_cfg);

    % =====================================================================
    % CONSTRAINTS
    % =====================================================================
    g = [];

    % Stall speed constraint
    g_stall = constraint_stall(aero_results, prob_cfg);
    g = [g, g_stall];

    % Water takeoff constraint
    g_takeoff = constraint_takeoff(amphi_results, amphi_cfg);
    g = [g, g_takeoff];

    % Power/energy feasibility
    g_power = constraint_power(prop_results);
    g = [g, g_power];

    % Geometric constraints (aspect ratio, etc.)
    g_geom = constraint_geometry(design_vars, prob_cfg);
    g = [g, g_geom];

    % =====================================================================
    % APPLY CONSTRAINT PENALTY (for single-objective or penalty approach)
    % =====================================================================
    % Note: In MOO with feasibility_first, constraints are handled separately.
    % The penalty is applied here for single-objective mode.

    % =====================================================================
    % ASSEMBLE OUTPUTS
    % =====================================================================
    obj_values = [f1, f2, f3];
    constraint_values = g;

    % Store all discipline data for post-processing
    discipline_data = struct();
    discipline_data.aero = aero_results;
    discipline_data.propulsion = prop_results;
    discipline_data.amphibious = amphi_results;
    discipline_data.objectives = struct('clcd', f1, 'weight', f2, 'range', f3);
    discipline_data.constraints = g;
    discipline_data.feasible = all(g <= 0);

end

%% ========================================================================
%  LOCAL CONSTRAINT FUNCTIONS
%  ========================================================================

function g = constraint_power(prop_results)
% CONSTRAINT_POWER  Power and energy feasibility constraints.
    % g1: Battery can deliver cruise power
    g1 = 0;
    if ~prop_results.feasible
        g1 = 1.0;
    end

    % g2: Thrust available >= thrust required
    T_avail = prop_results.thrust_cruise;
    T_req = prop_results.thrust_required;
    g2 = (T_req - T_avail) / max(T_req, 1);

    g = [g1, g2];
end

function g = constraint_geometry(design_vars, prob_cfg)
% CONSTRAINT_GEOMETRY  Geometric feasibility constraints.
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    % Aspect ratio limits (4 < AR < 14 for practical wings)
    S_ref = params.wing_span * (params.wing_chord_root + params.wing_chord_tip) / 2;
    AR = params.wing_span^2 / S_ref;
    g1 = (4 - AR) / 4;    % AR >= 4
    g2 = (AR - 14) / 14;  % AR <= 14

    % Taper ratio limits (0.2 < taper < 1.0)
    taper = params.wing_chord_tip / params.wing_chord_root;
    g3 = (0.2 - taper) / 0.2;  % taper >= 0.2
    g4 = (taper - 1.0);         % taper <= 1.0

    g = [g1, g2, g3, g4];
end
