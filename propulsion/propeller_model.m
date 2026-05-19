function prop = propeller_model(shaft_power, V_inf, rho, prop_cfg)
% PROPELLER_MODEL  Propeller performance and sizing model.
%   prop = propeller_model(shaft_power, V_inf, rho, prop_cfg)
%
%   Uses momentum theory + empirical corrections to estimate
%   propeller thrust, efficiency, and optimal diameter.
%
%   Inputs:
%     shaft_power - [W] Power delivered to propeller shaft
%     V_inf       - [m/s] Freestream velocity
%     rho         - [kg/m^3] Air density
%     prop_cfg    - Propulsion config struct (from propulsion_config.m)
%
%   Output:
%     prop - Struct with fields:
%            .thrust          - [N] Thrust produced
%            .efficiency      - [-] Propulsive efficiency (T*V / P_shaft)
%            .diameter        - [m] Propeller diameter
%            .rpm             - [rpm] Operating RPM
%            .advance_ratio   - [-] J = V / (n*D)
%            .tip_speed       - [m/s] Tip speed
%            .tip_mach        - [-] Tip Mach number
%            .mass            - [kg] Propeller mass estimate
%            .valid           - Logical (tip Mach within limits)

    cfg = prop_cfg.propeller;
    a_sound = 340;  % [m/s] Speed of sound (ISA sea level)

    % --- Propeller sizing ---
    % Optimal diameter from momentum theory:
    % P = T * V + T^(3/2) / (2*rho*A)^(1/2)  (simplified)
    % Use empirical sizing: D = k * P^(1/4) for electric props
    D_min = cfg.diameter_range(1);
    D_max = cfg.diameter_range(2);

    % Optimal diameter (empirical: ~1.2-1.6m per 100kW for GA aircraft)
    D_optimal = 0.55 * (shaft_power / 1000)^0.25;  % [m]
    D = max(D_min, min(D_max, D_optimal));

    % Disk area
    A_disk = pi * D^2 / 4;

    % --- Momentum theory for thrust ---
    % Actuator disk: T = 2 * rho * A * V_disk * (V_disk - V_inf)
    % Simplified: use efficiency to back out thrust
    if V_inf < 1.0
        % Static thrust condition
        T = (2 * rho * A_disk * shaft_power^2)^(1/3);
        eta = 0;  % Propulsive efficiency undefined at zero speed
    else
        % Cruise/climb: use efficiency model
        % Blade element considerations captured via advance ratio
        n_rps = V_inf / (0.7 * D);  % Approximate: J ~ 0.7 at peak efficiency
        J = V_inf / (n_rps * D);     % Advance ratio

        % Efficiency model (quadratic fit around J_opt)
        J_opt = 0.7;
        eta_max = cfg.efficiency_cruise;
        eta = eta_max * (1 - 2.5 * (J - J_opt)^2);
        eta = max(0.4, min(eta_max, eta));

        % Thrust from efficiency
        T = eta * shaft_power / V_inf;
    end

    % --- RPM calculation ---
    if V_inf < 1.0
        n_rps = sqrt(T / (rho * D^4 * 0.07));  % CT ~ 0.07 static
    end
    rpm = n_rps * 60;

    % --- Tip speed and Mach check ---
    V_tip = pi * D * n_rps;  % [m/s]
    helical_tip = sqrt(V_tip^2 + V_inf^2);
    tip_mach = helical_tip / a_sound;

    % --- Propeller mass (empirical) ---
    % Carbon fiber prop: ~0.3-0.5 kg/m of diameter per blade
    mass = cfg.n_blades * 0.4 * D;  % [kg]

    % --- Validity check ---
    valid = (tip_mach <= cfg.tip_mach_limit);

    % --- Build output struct ---
    prop = struct();
    prop.thrust         = T;
    prop.efficiency     = eta;
    prop.diameter       = D;
    prop.rpm            = rpm;
    prop.advance_ratio  = V_inf / (n_rps * D);
    prop.tip_speed      = V_tip;
    prop.tip_mach       = tip_mach;
    prop.mass           = mass;
    prop.valid          = valid;

end
