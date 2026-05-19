function f = objective_range(aero_results, prop_results, prob_cfg)
% OBJECTIVE_RANGE  Aircraft range objective.
%   f = objective_range(aero_results, prop_results, prob_cfg)
%
%   Computes the cruise range using the electric aircraft range equation.
%   The optimizer maximizes this (via obj_sense = -1).
%
%   The electric range equation (Breguet-equivalent):
%     R = E_usable * eta_total * (L/D) / W
%
%   Inputs:
%     aero_results - Struct from compute_aero()
%     prop_results - Struct from propulsion_analysis()
%     prob_cfg     - Problem config struct
%
%   Output:
%     f - Range [km] (higher is better)
%         Returns 0 if results are invalid.

    % --- Check validity ---
    if ~aero_results.valid
        f = 0;
        return;
    end

    % --- Extract parameters ---
    LD = aero_results.CL_CD;              % Lift-to-drag ratio
    E_usable = prop_results.battery.energy_usable;  % [Wh] Usable energy
    V_cruise = prob_cfg.mission.cruise_speed;       % [m/s]
    W = prob_cfg.aircraft.MTOW * prob_cfg.atmo.g;   % [N] Weight

    % --- Overall propulsive efficiency ---
    eta_motor = prop_results.motor.efficiency_cruise;
    eta_prop = prop_results.propeller.efficiency;
    eta_total = eta_motor * eta_prop;

    % --- Range calculation (electric Breguet) ---
    % R = (E * eta_total * L/D) / W
    % E in [J], W in [N], R in [m]
    E_J = E_usable * 3600;  % Convert Wh to J

    R_m = (E_J * eta_total * LD) / W;  % [m]
    R_km = R_m / 1000;                  % [km]

    % --- Alternative: simple V * endurance check ---
    % R_simple = V_cruise * prop_results.endurance_hr * 3.6;  % [km]

    % Use the more physically meaningful Breguet equation
    f = R_km;

    % Sanity bounds
    f = max(0, min(f, 2000));  % Cap at 2000 km (unrealistic above for electric)

end
