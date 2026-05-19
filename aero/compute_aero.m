function aero_results = compute_aero(design_vars, prob_cfg, amphi_cfg)
% COMPUTE_AERO  Compute aerodynamic coefficients for given design variables.
%   aero_results = compute_aero(design_vars, prob_cfg, amphi_cfg)
%
%   This function serves as the aerodynamic discipline wrapper:
%     - Calls run_openvsp() for VLM/Panel analysis if available
%     - Falls back to analytical estimation if OpenVSP is not installed
%     - Adds amphibious drag penalties (hull, step, spray rails)
%
%   Inputs:
%     design_vars - [1 x n_vars] Design variable vector
%     prob_cfg    - Problem config struct
%     amphi_cfg   - Amphibious config struct
%
%   Output:
%     aero_results - Struct with fields:
%                    .CL         - Lift coefficient at cruise AoA
%                    .CD         - Total drag coefficient at cruise AoA
%                    .CL_CD      - Lift-to-drag ratio
%                    .CL_max     - Maximum lift coefficient
%                    .alpha_stall- Stall angle [deg]
%                    .polar      - [n_alpha x 3] (alpha, CL, CD) table
%                    .S_ref      - Reference wing area [m^2]
%                    .method     - 'openvsp' or 'analytical'
%                    .valid      - Logical (true if results are physical)

    % --- Map design variables ---
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    % --- Compute wing reference geometry ---
    b = params.wing_span;
    c_root = params.wing_chord_root;
    c_tip = params.wing_chord_tip;
    S_ref = b * (c_root + c_tip) / 2;   % Trapezoidal wing area
    AR = b^2 / S_ref;                     % Aspect ratio
    taper = c_tip / c_root;               % Taper ratio

    % --- Try OpenVSP first ---
    use_openvsp = check_openvsp_available();

    if use_openvsp
        vsp_output = run_openvsp(design_vars, prob_cfg, []);
        if vsp_output.success
            raw_polar = parse_vsp_results(vsp_output.polar_file);
            aero_results = build_aero_from_polar(raw_polar, params, amphi_cfg, S_ref, AR);
            aero_results.method = 'openvsp';
            return;
        end
        % If OpenVSP fails, fall through to analytical
        warning('compute_aero: OpenVSP failed, using analytical fallback.');
    end

    % --- Analytical estimation (fallback / standalone mode) ---
    aero_results = analytical_aero(params, prob_cfg, amphi_cfg, S_ref, AR, taper);
    aero_results.method = 'analytical';

end

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function available = check_openvsp_available()
% CHECK_OPENVSP_AVAILABLE  Check if OpenVSP executables are on the path.
    [status, ~] = system('vsp -version');
    available = (status == 0);
end

function aero = build_aero_from_polar(polar_data, params, amphi_cfg, S_ref, AR)
% BUILD_AERO_FROM_POLAR  Process VSPAero polar into structured results.

    alpha = polar_data(:, 1);
    CL = polar_data(:, 2);
    CD = polar_data(:, 3);

    % Add amphibious drag penalties
    dCd0 = amphi_cfg.drag.hull_Cd0_increment + ...
            amphi_cfg.drag.step_Cd0_increment + ...
            amphi_cfg.drag.spray_rail_Cd0;
    CD = CD + dCd0;

    % Find cruise point (CL for level flight)
    W = 1500 * 9.81;  % MTOW * g  (will be passed properly in full integration)
    rho = 1.225;
    V = 55;  % cruise speed
    CL_cruise = (2 * W) / (rho * V^2 * S_ref);

    % Interpolate CD at cruise CL
    if CL_cruise >= min(CL) && CL_cruise <= max(CL)
        CD_cruise = interp1(CL, CD, CL_cruise, 'pchip');
    else
        CD_cruise = CD(round(end/2));  % Fallback to mid-alpha
        CL_cruise = CL(round(end/2));
    end

    % CL_max and stall angle
    [CL_max, idx_max] = max(CL);
    alpha_stall = alpha(idx_max);

    aero = struct();
    aero.CL         = CL_cruise;
    aero.CD         = CD_cruise;
    aero.CL_CD      = CL_cruise / CD_cruise;
    aero.CL_max     = CL_max;
    aero.alpha_stall = alpha_stall;
    aero.polar      = [alpha, CL, CD];
    aero.S_ref      = S_ref;
    aero.valid       = (CL_cruise > 0 && CD_cruise > 0 && AR > 4);
end

function aero = analytical_aero(params, prob_cfg, amphi_cfg, S_ref, AR, taper)
% ANALYTICAL_AERO  Semi-empirical aerodynamic estimation.
%   Uses lifting-line theory + flat plate drag buildup.

    % --- Lift curve slope (Helmbold equation for finite wings) ---
    CL_alpha_2d = 2 * pi;  % [1/rad] Thin airfoil theory
    sweep_rad = deg2rad(params.wing_sweep);
    cos_sweep = cos(sweep_rad);

    % Helmbold correction for finite wing
    CL_alpha = CL_alpha_2d * AR / ...
        (2 + sqrt(4 + AR^2 * (1 + tan(sweep_rad)^2)));  % [1/rad]

    % --- Cruise condition ---
    W = prob_cfg.aircraft.MTOW * prob_cfg.atmo.g;
    rho = prob_cfg.atmo.rho_sl;
    V = prob_cfg.mission.cruise_speed;
    q = 0.5 * rho * V^2;

    CL_cruise = W / (q * S_ref);
    alpha_cruise = CL_cruise / CL_alpha;  % [rad]

    % --- CL_max estimation ---
    % Empirical: depends on t/c and flap configuration
    tc = params.wing_tc_ratio;
    CL_max = 1.2 + 3.0 * tc;  % Simplified (no flaps)

    alpha_stall = CL_max / CL_alpha;  % [rad]

    % --- Drag estimation (component buildup) ---
    % Parasitic drag
    Cd0_wing = estimate_skin_friction(params, prob_cfg, S_ref);
    Cd0_base = prob_cfg.aircraft.Cd0_base;

    % Amphibious drag penalties
    dCd0_amphi = amphi_cfg.drag.hull_Cd0_increment + ...
                 amphi_cfg.drag.step_Cd0_increment + ...
                 amphi_cfg.drag.spray_rail_Cd0;

    Cd0_total = Cd0_base + dCd0_amphi;

    % Induced drag (Oswald efficiency)
    e = oswald_efficiency(AR, taper, params.wing_sweep);
    Cdi = CL_cruise^2 / (pi * e * AR);

    % Total drag at cruise
    CD_cruise = Cd0_total + Cdi;

    % --- Build alpha sweep (for polar output) ---
    alpha_deg = linspace(-2, 14, 17)';
    CL_sweep = CL_alpha * deg2rad(alpha_deg);
    CD_sweep = Cd0_total + CL_sweep.^2 / (pi * e * AR);

    % Cap at stall
    CL_sweep = min(CL_sweep, CL_max);

    % --- Assemble output ---
    aero = struct();
    aero.CL          = CL_cruise;
    aero.CD          = CD_cruise;
    aero.CL_CD       = CL_cruise / CD_cruise;
    aero.CL_max      = CL_max;
    aero.alpha_stall = rad2deg(alpha_stall);
    aero.polar       = [alpha_deg, CL_sweep, CD_sweep];
    aero.S_ref       = S_ref;
    aero.valid       = (CL_cruise > 0 && CD_cruise > 0.005 && AR > 4);
end

function Cf = estimate_skin_friction(params, prob_cfg, S_ref)
% ESTIMATE_SKIN_FRICTION  Flat plate skin friction for wing.
    Re = prob_cfg.atmo.rho_sl * prob_cfg.mission.cruise_speed * ...
         (params.wing_chord_root + params.wing_chord_tip)/2 / prob_cfg.atmo.mu;
    % Turbulent flat plate (Schlichting)
    Cf = 0.455 / (log10(Re))^2.58;
    % Form factor correction (DATCOM)
    tc = params.wing_tc_ratio;
    FF = 1 + 2.0*tc + 60*tc^4;
    Cf = Cf * FF;
end

function e = oswald_efficiency(AR, taper, sweep_deg)
% OSWALD_EFFICIENCY  Estimate Oswald efficiency factor.
%   Empirical correlation (Raymer-style).
    sweep_rad = deg2rad(sweep_deg);
    % Nita & Scholz (2012) correlation
    e_theory = 1 / (1 + 0.0075 * AR);  % Simplified
    % Correction for taper and sweep
    k_taper = 0.95;  % Near optimal for taper = 0.3-0.5
    if taper > 0.5
        k_taper = 0.92;
    end
    k_sweep = 1 - 0.002 * sweep_deg;  % Mild sweep penalty
    e = e_theory * k_taper * k_sweep;
    e = max(0.6, min(0.95, e));  % Physical bounds
end
