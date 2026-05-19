function amphi_results = amphibious_analysis(design_vars, prob_cfg, prop_cfg, amphi_cfg, aero_results, prop_results)
% AMPHIBIOUS_ANALYSIS  Integrated amphibious performance analysis.
%   amphi_results = amphibious_analysis(design_vars, prob_cfg, prop_cfg, amphi_cfg, aero_results, prop_results)
%
%   Master function for all amphibious-specific calculations:
%     - Hull hydrodynamic performance
%     - Water takeoff distance and feasibility
%     - Float/hull penalties (drag and weight)
%     - Stability assessment
%
%   Inputs:
%     design_vars  - [1 x n_vars] Design variable vector
%     prob_cfg     - Problem config struct
%     prop_cfg     - Propulsion config struct
%     amphi_cfg    - Amphibious config struct
%     aero_results - Struct from compute_aero()
%     prop_results - Struct from propulsion_analysis()
%
%   Output:
%     amphi_results - Struct with fields:
%                     .hydro         - Hydrodynamic results at cruise-on-water
%                     .takeoff       - Water takeoff results
%                     .penalties     - Drag and weight penalty struct
%                     .landing       - Water landing assessment
%                     .feasible      - Overall amphibious feasibility
%                     .constraint_violations - [1 x n] constraint violation values

    % --- Map design variables ---
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    % --- Hydrodynamic analysis at representative speed ---
    % Use 50% of liftoff speed as representative water cruise
    V_stall = sqrt(2 * prob_cfg.aircraft.MTOW * prob_cfg.atmo.g / ...
              (prob_cfg.atmo.rho_sl * aero_results.S_ref * aero_results.CL_max));
    V_water_cruise = 0.5 * V_stall * amphi_cfg.takeoff.rotation_speed_factor;

    hydro = hull_hydro(V_water_cruise, params, prob_cfg, amphi_cfg);

    % --- Water takeoff analysis ---
    takeoff = water_takeoff(design_vars, prob_cfg, prop_cfg, amphi_cfg, ...
                            aero_results, prop_results);

    % --- Float/hull penalties ---
    penalties = float_penalties(params, amphi_cfg);

    % --- Water landing assessment ---
    landing = assess_water_landing(params, prob_cfg, amphi_cfg, aero_results);

    % --- Constraint violations ---
    constraints = evaluate_amphi_constraints(takeoff, hydro, landing, amphi_cfg);

    % --- Overall feasibility ---
    feasible = takeoff.feasible && ...
               hydro.porpoising_stable && ...
               landing.feasible && ...
               all(constraints <= 0);

    % --- Build output struct ---
    amphi_results = struct();
    amphi_results.hydro       = hydro;
    amphi_results.takeoff     = takeoff;
    amphi_results.penalties   = penalties;
    amphi_results.landing     = landing;
    amphi_results.feasible    = feasible;
    amphi_results.constraint_violations = constraints;

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function landing = assess_water_landing(params, prob_cfg, amphi_cfg, aero_results)
% ASSESS_WATER_LANDING  Check water landing feasibility.
    land_cfg = amphi_cfg.landing;

    % Approach speed
    V_stall = sqrt(2 * prob_cfg.aircraft.MTOW * prob_cfg.atmo.g / ...
              (prob_cfg.atmo.rho_sl * aero_results.S_ref * aero_results.CL_max));
    V_approach = V_stall * land_cfg.approach_speed_factor;

    % Sink rate check
    % Typical: descent angle ~ 3-5 deg for water landing
    gamma_approach = 3;  % [deg] approach angle
    sink_rate = V_approach * sin(deg2rad(gamma_approach));

    % Load factor at impact
    % Simplified: n = 1 + (sink_rate / V_vertical_ref)^2
    V_ref = 1.0;  % Reference vertical speed for n=2
    n_landing = 1 + (sink_rate / V_ref)^2;

    % Hull beam adequacy for landing loads
    beam = params.hull_beam;
    beam_loading = prob_cfg.aircraft.MTOW * prob_cfg.atmo.g * n_landing / beam^2;
    % Typical limit: ~50000 N/m^2 for composite hull
    beam_adequate = (beam_loading < 50000);

    % Feasibility
    sink_ok = (sink_rate <= land_cfg.max_sink_rate);
    load_ok = (n_landing <= land_cfg.load_factor);
    feasible = sink_ok && load_ok && beam_adequate;

    landing = struct();
    landing.V_approach    = V_approach;
    landing.sink_rate     = sink_rate;
    landing.load_factor   = n_landing;
    landing.beam_loading  = beam_loading;
    landing.feasible      = feasible;
end

function g = evaluate_amphi_constraints(takeoff, hydro, landing, amphi_cfg)
% EVALUATE_AMPHI_CONSTRAINTS  Compute constraint violations (g <= 0 = feasible).
    % g1: Takeoff distance <= max allowed
    g1 = (takeoff.distance - amphi_cfg.takeoff.max_water_run) / amphi_cfg.takeoff.max_water_run;

    % g2: Thrust margin at hump > 1.0
    g2 = (1.0 - takeoff.thrust_margin);

    % g3: Porpoising stability
    g3 = 0;
    if ~hydro.porpoising_stable
        g3 = 1.0;  % Violated
    end

    % g4: Landing load factor <= limit
    g4 = (landing.load_factor - amphi_cfg.landing.load_factor) / amphi_cfg.landing.load_factor;

    % g5: Sink rate <= limit
    g5 = (landing.sink_rate - amphi_cfg.landing.max_sink_rate) / amphi_cfg.landing.max_sink_rate;

    g = [g1, g2, g3, g4, g5];
end
