function cfg = propulsion_config()
% PROPULSION_CONFIG  Electric propulsion system parameters.
%   Battery, motor, and propeller specifications for the electric aircraft.

    cfg = struct();

    % =========================================================================
    % BATTERY
    % =========================================================================
    cfg.battery.specific_energy     = 250;     % [Wh/kg] Battery specific energy (cell level)
    cfg.battery.pack_efficiency     = 0.85;    % [-] Pack-level efficiency (cell-to-pack)
    cfg.battery.depth_of_discharge  = 0.80;    % [-] Usable DoD
    cfg.battery.charge_efficiency   = 0.95;    % [-] Charge/discharge efficiency
    cfg.battery.min_reserve         = 0.10;    % [-] Minimum reserve fraction (safety)
    cfg.battery.degradation_factor  = 0.90;    % [-] End-of-life capacity factor
    cfg.battery.voltage_nominal     = 400;     % [V] Nominal pack voltage
    cfg.battery.mass_overhead       = 1.15;    % [-] BMS + cabling + thermal mass multiplier

    % =========================================================================
    % MOTOR
    % =========================================================================
    cfg.motor.efficiency_peak       = 0.95;    % [-] Peak motor efficiency
    cfg.motor.efficiency_cruise     = 0.92;    % [-] Cruise-point efficiency
    cfg.motor.specific_power        = 5.0;     % [kW/kg] Motor specific power
    cfg.motor.power_factor          = 0.90;    % [-] Power electronics efficiency
    cfg.motor.cooling_mass_frac     = 0.10;    % [-] Cooling system mass / motor mass

    % =========================================================================
    % PROPELLER
    % =========================================================================
    cfg.propeller.n_blades          = 3;       % Number of blades
    cfg.propeller.diameter_range    = [1.5, 2.5]; % [m] Propeller diameter bounds
    cfg.propeller.efficiency_cruise = 0.82;    % [-] Propulsive efficiency at cruise
    cfg.propeller.efficiency_climb  = 0.75;    % [-] Propulsive efficiency at climb
    cfg.propeller.tip_mach_limit    = 0.70;    % [-] Max tip Mach number

    % =========================================================================
    % POWER MANAGEMENT
    % =========================================================================
    cfg.power.climb_power_fraction  = 0.85;    % [-] Fraction of max power used in climb
    cfg.power.cruise_power_fraction = 0.55;    % [-] Fraction of max power used in cruise
    cfg.power.avionics_power        = 1.5;     % [kW] Constant avionics/systems draw
    cfg.power.margin                = 1.10;    % [-] Power sizing margin (10% reserve)

end
