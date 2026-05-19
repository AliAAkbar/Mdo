function cfg = problem_config()
% PROBLEM_CONFIG  Design variable definitions, bounds, and mission parameters.
%   Defines the optimization problem for the electric amphibious aircraft.

    cfg = struct();

    % =========================================================================
    % DESIGN VARIABLES
    % Each row: [lower_bound, upper_bound]
    % =========================================================================

    % --- Wing geometry ---
    cfg.var_names = {
        'wing_span',        ...  % [m] Wing span
        'wing_chord_root',  ...  % [m] Root chord
        'wing_chord_tip',   ...  % [m] Tip chord
        'wing_sweep',       ...  % [deg] Leading edge sweep
        'wing_twist',       ...  % [deg] Tip twist (washout negative)
        'wing_dihedral',    ...  % [deg] Dihedral angle
        'wing_tc_ratio',    ...  % [-] Thickness-to-chord ratio
        'hull_beam',        ...  % [m] Hull beam width
        'hull_deadrise',    ...  % [deg] Hull deadrise angle
        'battery_mass_frac',...  % [-] Battery mass fraction of MTOW
        'motor_power'       ...  % [kW] Motor rated power
    };

    cfg.lb = [8.0,  1.0, 0.4,  0,  -5,  0,  0.10, 1.0, 15, 0.20, 50 ];
    cfg.ub = [16.0, 2.5, 1.2, 20,   0, 10,  0.18, 2.5, 30, 0.45, 200];

    cfg.n_vars = numel(cfg.lb);

    % =========================================================================
    % MISSION PARAMETERS
    % =========================================================================
    cfg.mission.cruise_speed    = 55;      % [m/s] Cruise speed
    cfg.mission.cruise_altitude = 1000;    % [m] Cruise altitude
    cfg.mission.range_required  = 200;     % [km] Minimum range requirement
    cfg.mission.payload_mass    = 200;     % [kg] Payload
    cfg.mission.n_passengers    = 4;       % Number of passengers (incl. pilot)

    % =========================================================================
    % AIRCRAFT PARAMETERS (fixed)
    % =========================================================================
    cfg.aircraft.MTOW           = 1500;    % [kg] Maximum takeoff weight
    cfg.aircraft.empty_weight_frac = 0.35; % [-] Structural empty weight / MTOW
    cfg.aircraft.Cd0_base       = 0.025;   % [-] Parasitic drag coefficient (base)

    % =========================================================================
    % ATMOSPHERIC CONDITIONS (ISA sea level defaults)
    % =========================================================================
    cfg.atmo.rho_sl     = 1.225;   % [kg/m^3] Air density at sea level
    cfg.atmo.g          = 9.81;    % [m/s^2] Gravitational acceleration
    cfg.atmo.mu         = 1.789e-5;% [Pa.s] Dynamic viscosity

    % =========================================================================
    % OPTIMIZATION MODE
    % =========================================================================
    cfg.mode = 'multi';  % 'single' or 'multi'

end
