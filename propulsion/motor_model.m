function motor = motor_model(rated_power_kW, prop_cfg)
% MOTOR_MODEL  Electric motor sizing and performance model.
%   motor = motor_model(rated_power_kW, prop_cfg)
%
%   Models a permanent magnet synchronous motor (PMSM) with
%   power electronics (inverter/controller).
%
%   Inputs:
%     rated_power_kW - [kW] Motor rated (maximum continuous) power
%     prop_cfg       - Propulsion config struct (from propulsion_config.m)
%
%   Output:
%     motor - Struct with fields:
%             .rated_power      - [W] Rated power
%             .mass_motor       - [kg] Motor mass (incl. cooling)
%             .mass_controller  - [kg] Power electronics mass
%             .mass_total       - [kg] Total drivetrain mass (motor + controller)
%             .efficiency_peak  - [-] Peak efficiency
%             .efficiency_cruise- [-] Efficiency at cruise power
%             .power_cruise     - [W] Power output at cruise
%             .power_climb      - [W] Power output at climb
%             .heat_rejection   - [W] Waste heat at cruise (for thermal sizing)

    cfg = prop_cfg.motor;
    pwr_cfg = prop_cfg.power;

    % --- Rated power in Watts ---
    P_rated = rated_power_kW * 1000;  % [W]

    % --- Motor mass ---
    mass_motor_bare = P_rated / (cfg.specific_power * 1000);  % [kg]
    mass_cooling = mass_motor_bare * cfg.cooling_mass_frac;
    mass_motor = mass_motor_bare + mass_cooling;

    % --- Power electronics (inverter/controller) ---
    % Typical: ~0.5-1.0 kg per 10 kW for aerospace-grade
    specific_power_controller = 10.0;  % [kW/kg] for modern SiC inverter
    mass_controller = rated_power_kW / specific_power_controller;

    mass_total = mass_motor + mass_controller;

    % --- Efficiency at operating points ---
    % Motor efficiency varies with load fraction
    % Model: eta = eta_peak * (1 - k * (1 - load_fraction)^2)
    eta_peak = cfg.efficiency_peak;
    eta_factor = cfg.power_factor;  % Power electronics efficiency

    % Cruise power
    P_cruise = P_rated * pwr_cfg.cruise_power_fraction;
    load_frac_cruise = pwr_cfg.cruise_power_fraction;
    eta_motor_cruise = eta_peak * (1 - 0.03 * (1 - load_frac_cruise)^2);
    eta_cruise = eta_motor_cruise * eta_factor;

    % Climb power
    P_climb = P_rated * pwr_cfg.climb_power_fraction;
    load_frac_climb = pwr_cfg.climb_power_fraction;
    eta_motor_climb = eta_peak * (1 - 0.03 * (1 - load_frac_climb)^2);
    eta_climb = eta_motor_climb * eta_factor;

    % --- Thermal: waste heat at cruise ---
    heat_rejection_cruise = P_cruise * (1 - eta_cruise);

    % --- Build output struct ---
    motor = struct();
    motor.rated_power       = P_rated;
    motor.mass_motor        = mass_motor;
    motor.mass_controller   = mass_controller;
    motor.mass_total        = mass_total;
    motor.efficiency_peak   = eta_peak;
    motor.efficiency_cruise = eta_cruise;
    motor.efficiency_climb  = eta_climb;
    motor.power_cruise      = P_cruise;
    motor.power_climb       = P_climb;
    motor.heat_rejection    = heat_rejection_cruise;

end
