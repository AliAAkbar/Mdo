function hydro = hull_hydro(V_water, params, prob_cfg, amphi_cfg)
% HULL_HYDRO  Hull hydrodynamic resistance model for planing hull.
%   hydro = hull_hydro(V_water, params, prob_cfg, amphi_cfg)
%
%   Computes water resistance (drag) for a planing flying-boat hull
%   using Savitsky's method adapted for seaplane hulls.
%
%   Inputs:
%     V_water   - [m/s] Water speed (boat speed)
%     params    - Struct with design variable fields (hull_beam, hull_deadrise, etc.)
%     prob_cfg  - Problem config struct
%     amphi_cfg - Amphibious config struct
%
%   Output:
%     hydro - Struct with fields:
%             .resistance_total - [N] Total water resistance
%             .resistance_friction - [N] Skin friction resistance
%             .resistance_pressure - [N] Pressure (wave-making) resistance
%             .resistance_spray    - [N] Spray resistance
%             .lift_hydro       - [N] Hydrodynamic lift force
%             .wetted_area      - [m^2] Wetted hull area
%             .Cv               - [-] Speed coefficient
%             .trim_angle       - [deg] Running trim angle
%             .porpoising_stable - Logical (true if stable)

    cfg = amphi_cfg.hydro;
    hull_cfg = amphi_cfg.hull;

    % --- Hull geometry ---
    beam = params.hull_beam;                         % [m]
    deadrise = params.hull_deadrise;                 % [deg]
    hull_length = beam * hull_cfg.length_beam_ratio; % [m]
    forebody_len = hull_length * hull_cfg.forebody_fraction;

    % --- Water properties ---
    rho_w = cfg.water_density;    % [kg/m^3]
    g = prob_cfg.atmo.g;

    % --- Speed coefficient (Cv) ---
    % Cv = V / sqrt(g * beam)
    Cv = V_water / sqrt(g * beam);
    Cv = Cv * cfg.Cv_factor;  % Correction factor

    % --- Operating regime ---
    % Cv < 0.5: displacement, 0.5-1.5: transition, >1.5: planing
    if Cv < 0.1
        % Nearly stationary
        hydro = static_buoyancy(params, prob_cfg, amphi_cfg);
        return;
    end

    % --- Savitsky planing method ---
    W = prob_cfg.aircraft.MTOW * g;  % [N] Weight

    % Trim angle estimation (empirical for seaplane hulls)
    % At hump speed (Cv ~ 1-2): trim ~ 6-8 deg
    % At takeoff speed (Cv ~ 3-5): trim ~ 3-5 deg
    if Cv < 1.5
        tau = 6 + 2 * (1 - Cv/1.5);  % Higher trim at low speed
    else
        tau = 3 + 3 / Cv;  % Decreasing trim as speed increases
    end
    tau = max(2, min(10, tau));  % Physical limits [deg]

    % --- Wetted length and area ---
    % Mean wetted length ratio (lambda = L_wetted / beam)
    % Savitsky: CL = tau^1.1 * (0.0120*lambda^0.5 + 0.0055*lambda^2.5 / Cv^2)
    % Solve for lambda given CL_hydro (lift coefficient)
    CL_hydro = W / (0.5 * rho_w * V_water^2 * beam^2);

    % Iterative solve for wetted length
    lambda = solve_wetted_length(CL_hydro, tau, Cv, deadrise);

    L_wetted = lambda * beam;
    wetted_area = L_wetted * beam;  % Simplified rectangular approximation

    % --- Friction resistance (Schoenherr) ---
    Re_hull = rho_w * V_water * L_wetted / 1.07e-3;  % Re (seawater viscosity ~1.07e-3 Pa.s)
    if Re_hull > 1e4
        Cf = 0.075 / (log10(Re_hull) - 2)^2;  % ITTC 1957 line
    else
        Cf = cfg.friction_coeff;  % Use configured value for low Re
    end

    R_friction = 0.5 * rho_w * V_water^2 * wetted_area * Cf / cos(deg2rad(tau));

    % --- Pressure (wave-making) resistance ---
    % Savitsky: R_pressure = W * tan(tau) - hydrodynamic lift vertical component
    % Simplified:
    R_pressure = W * tan(deg2rad(tau)) * (1 - Cv/10);
    R_pressure = max(0, R_pressure);
    R_pressure = R_pressure * cfg.wave_drag_factor;

    % --- Spray resistance ---
    % Empirical: proportional to speed squared at high Cv
    R_spray = 0.5 * rho_w * V_water^2 * beam * 0.003 * Cv;  % Simplified
    R_spray = R_spray * cfg.spray_drag_factor;

    % --- Total resistance ---
    R_total = R_friction + R_pressure + R_spray;

    % --- Hydrodynamic lift ---
    lift_hydro = 0.5 * rho_w * V_water^2 * beam^2 * CL_hydro;

    % --- Porpoising stability check ---
    % Savitsky stability criterion: unstable if trim > critical
    tau_crit = 15 - 2 * Cv;  % Simplified criterion
    tau_crit = max(3, tau_crit);
    porpoising_stable = (tau < tau_crit / cfg.porpoising_margin);

    % --- Build output struct ---
    hydro = struct();
    hydro.resistance_total    = R_total;
    hydro.resistance_friction = R_friction;
    hydro.resistance_pressure = R_pressure;
    hydro.resistance_spray    = R_spray;
    hydro.lift_hydro          = lift_hydro;
    hydro.wetted_area         = wetted_area;
    hydro.Cv                  = Cv;
    hydro.trim_angle          = tau;
    hydro.porpoising_stable   = porpoising_stable;

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function lambda = solve_wetted_length(CL_target, tau, Cv, deadrise)
% SOLVE_WETTED_LENGTH  Solve for wetted length ratio using Savitsky equations.
    % Simplified iterative approach
    % CL = tau^1.1 * (0.012*sqrt(lambda) + 0.0055*lambda^2.5/Cv^2)
    % Account for deadrise: CL_beta = CL_0 - 0.0065*beta*CL_0^0.6

    lambda = 2.0;  % Initial guess
    for iter = 1:20
        CL_0 = tau^1.1 * (0.012 * sqrt(lambda) + 0.0055 * lambda^2.5 / max(Cv^2, 0.01));
        CL_beta = CL_0 - 0.0065 * deadrise * CL_0^0.6;

        if CL_beta < 1e-10
            lambda = lambda * 1.5;
            continue;
        end

        % Newton-like update
        ratio = CL_target / CL_beta;
        lambda = lambda * ratio^0.4;  % Damped update
        lambda = max(0.5, min(20, lambda));

        if abs(ratio - 1) < 0.01
            break;
        end
    end
end

function hydro = static_buoyancy(params, prob_cfg, amphi_cfg)
% STATIC_BUOYANCY  Static floating condition (V ~ 0).
    beam = params.hull_beam;
    hull_length = beam * amphi_cfg.hull.length_beam_ratio;
    rho_w = amphi_cfg.hydro.water_density;
    g = prob_cfg.atmo.g;
    W = prob_cfg.aircraft.MTOW * g;

    % Draft from Archimedes: W = rho_w * g * Volume_submerged
    % Approximate hull as prismatic: V = beam * draft * length * 0.5 (Cb ~ 0.5)
    Cb = 0.5;  % Block coefficient
    draft = W / (rho_w * g * beam * hull_length * Cb);

    hydro = struct();
    hydro.resistance_total    = 0;
    hydro.resistance_friction = 0;
    hydro.resistance_pressure = 0;
    hydro.resistance_spray    = 0;
    hydro.lift_hydro          = W;  % Buoyancy supports weight
    hydro.wetted_area         = beam * hull_length * 0.6;  % Approx
    hydro.Cv                  = 0;
    hydro.trim_angle          = 0;
    hydro.porpoising_stable   = true;
end
