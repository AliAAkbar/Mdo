function cfg = amphibious_config()
% AMPHIBIOUS_CONFIG  Amphibious aircraft configuration parameters.
%   Hull hydrodynamics, water takeoff, and float/step geometry.

    cfg = struct();

    % =========================================================================
    % HULL GEOMETRY
    % =========================================================================
    cfg.hull.length_beam_ratio  = 6.0;     % [-] Hull length / beam ratio
    cfg.hull.forebody_fraction  = 0.55;    % [-] Forebody length / total hull length
    cfg.hull.afterbody_angle    = 7.0;     % [deg] Afterbody keel angle
    cfg.hull.step_height_ratio  = 0.05;    % [-] Step height / beam
    cfg.hull.bow_height_ratio   = 0.20;    % [-] Bow height / beam (spray control)

    % =========================================================================
    % HYDRODYNAMICS
    % =========================================================================
    cfg.hydro.water_density     = 1025;    % [kg/m^3] Seawater density
    cfg.hydro.Cv_factor         = 1.0;     % [-] Speed coefficient correction
    cfg.hydro.friction_coeff    = 0.004;   % [-] Hull skin friction coefficient
    cfg.hydro.spray_drag_factor = 1.15;    % [-] Spray drag multiplier
    cfg.hydro.wave_drag_factor  = 1.10;    % [-] Wave-making drag multiplier (calm water)
    cfg.hydro.porpoising_margin = 1.20;    % [-] Safety margin for porpoising stability

    % =========================================================================
    % WATER TAKEOFF
    % =========================================================================
    cfg.takeoff.max_water_run   = 500;     % [m] Maximum water takeoff distance
    cfg.takeoff.rotation_speed_factor = 1.15; % [-] V_rotation / V_stall ratio
    cfg.takeoff.ground_effect_factor  = 0.80; % [-] Induced drag reduction in ground effect
    cfg.takeoff.wind_speed      = 0;       % [m/s] Headwind for takeoff (0 = no wind)
    cfg.takeoff.wave_height     = 0.3;     % [m] Significant wave height (calm = 0-0.3m)
    cfg.takeoff.sea_state       = 2;       % Sea state (0=calm, 3=moderate)

    % =========================================================================
    % FLOAT / SPONSON (if applicable)
    % =========================================================================
    cfg.floats.type             = 'hull';  % 'hull', 'floats', or 'sponsons'
    cfg.floats.wetted_area_add  = 0;       % [m^2] Additional wetted area (0 for hull type)
    cfg.floats.drag_increment   = 0;       % [-] Delta_Cd0 from floats (0 for hull type)
    cfg.floats.weight_fraction  = 0.05;    % [-] Float/hull structure weight / MTOW

    % =========================================================================
    % WATER LANDING
    % =========================================================================
    cfg.landing.approach_speed_factor = 1.3;  % [-] V_approach / V_stall
    cfg.landing.max_sink_rate   = 1.5;     % [m/s] Maximum water sink rate at touchdown
    cfg.landing.load_factor     = 4.0;     % [-] Water landing load factor (g's)

    % =========================================================================
    % AIRBORNE DRAG PENALTIES (from amphibious features)
    % =========================================================================
    cfg.drag.hull_Cd0_increment = 0.005;   % [-] Additional Cd0 from hull shape
    cfg.drag.step_Cd0_increment = 0.002;   % [-] Additional Cd0 from hull step
    cfg.drag.spray_rail_Cd0     = 0.001;   % [-] Additional Cd0 from spray rails

end
