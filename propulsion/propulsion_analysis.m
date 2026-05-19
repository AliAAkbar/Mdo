function prop_results = propulsion_analysis(design_vars, prob_cfg, prop_cfg)
% PROPULSION_ANALYSIS  Integrated electric propulsion system analysis.
%   prop_results = propulsion_analysis(design_vars, prob_cfg, prop_cfg)
%
%   Combines battery, motor, and propeller models to evaluate the complete
%   electric propulsion system. Computes range, endurance, and power budget.
%
%   Inputs:
%     design_vars - [1 x n_vars] Design variable vector
%     prob_cfg    - Problem config struct (from problem_config.m)
%     prop_cfg    - Propulsion config struct (from propulsion_config.m)
%
%   Output:
%     prop_results - Struct with fields:
%                    .battery      - Battery model output struct
%                    .motor        - Motor model output struct
%                    .propeller    - Propeller model output struct
%                    .range_km     - [km] Estimated cruise range
%                    .endurance_hr - [hr] Estimated endurance at cruise
%                    .thrust_cruise- [N] Available thrust at cruise
%                    .thrust_required - [N] Required thrust at cruise (from drag)
%                    .power_budget - Struct with power breakdown
%                    .mass_total   - [kg] Total propulsion system mass
%                    .feasible     - Logical (true if power/energy sufficient)

    % --- Map design variables ---
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    % --- Extract propulsion design variables ---
    battery_mass_frac = params.battery_mass_frac;
    motor_power_kW = params.motor_power;
    MTOW = prob_cfg.aircraft.MTOW;

    battery_mass = battery_mass_frac * MTOW;  % [kg]

    % --- Flight conditions ---
    V_cruise = prob_cfg.mission.cruise_speed;
    rho = prob_cfg.atmo.rho_sl;  % Simplified: use sea-level for now
    g = prob_cfg.atmo.g;

    % --- Battery model ---
    battery = battery_model(battery_mass, prop_cfg);

    % --- Motor model ---
    motor = motor_model(motor_power_kW, prop_cfg);

    % --- Power at cruise ---
    P_shaft_cruise = motor.power_cruise * motor.efficiency_cruise;
    % Shaft power delivered to propeller after motor losses
    P_to_prop = motor.power_cruise;  % Motor output = shaft input to prop

    % --- Propeller model (cruise condition) ---
    propeller = propeller_model(P_to_prop, V_cruise, rho, prop_cfg);

    % --- Thrust available vs required ---
    thrust_available = propeller.thrust;

    % Required thrust (will be refined when coupled with aero)
    % Approximate: T_req = D = W / (L/D)
    % Use a placeholder L/D; the full MDO loop will couple this properly
    LD_estimate = 12;  % Conservative for amphibious
    W = MTOW * g;
    thrust_required = W / LD_estimate;

    % --- Power budget ---
    P_avionics = prop_cfg.power.avionics_power * 1000;  % [W]
    P_total_cruise = motor.power_cruise / motor.efficiency_cruise + P_avionics;
    % Total electrical power draw from battery at cruise

    % --- Range and endurance calculation ---
    % Energy available for propulsion (subtract avionics)
    E_usable_Wh = battery.energy_usable;
    E_usable_J = E_usable_Wh * 3600;  % Convert to Joules

    % Endurance at cruise power
    if P_total_cruise > 0
        endurance_hr = E_usable_Wh / P_total_cruise;
    else
        endurance_hr = 0;
    end

    % Range (simple Breguet-equivalent for electric)
    % R = E_usable * eta_total * (L/D) / (W)
    % Or simply R = V_cruise * endurance
    range_km = V_cruise * endurance_hr * 3.6;  % Convert hr*m/s to km
    % Note: 1 m/s * 1 hr = 3600 m = 3.6 km

    % --- Total propulsion system mass ---
    mass_propulsion = battery.mass_total + motor.mass_total + propeller.mass;

    % --- Feasibility checks ---
    power_feasible = (battery.power_max >= P_total_cruise);
    thrust_feasible = (thrust_available >= thrust_required * 0.9);  % 90% margin
    prop_feasible = propeller.valid;  % Tip Mach check
    feasible = power_feasible && thrust_feasible && prop_feasible;

    % --- Build output struct ---
    prop_results = struct();
    prop_results.battery        = battery;
    prop_results.motor          = motor;
    prop_results.propeller      = propeller;
    prop_results.range_km       = range_km;
    prop_results.endurance_hr   = endurance_hr;
    prop_results.thrust_cruise  = thrust_available;
    prop_results.thrust_required = thrust_required;
    prop_results.mass_total     = mass_propulsion;
    prop_results.feasible       = feasible;

    % Power budget breakdown
    prop_results.power_budget = struct();
    prop_results.power_budget.P_motor_input   = motor.power_cruise / motor.efficiency_cruise;
    prop_results.power_budget.P_motor_output  = motor.power_cruise;
    prop_results.power_budget.P_shaft         = P_to_prop;
    prop_results.power_budget.P_avionics      = P_avionics;
    prop_results.power_budget.P_total_battery = P_total_cruise;
    prop_results.power_budget.P_propulsive    = thrust_available * V_cruise;

end
