function g = constraint_stall(aero_results, prob_cfg)
% CONSTRAINT_STALL  Stall speed constraint.
%   g = constraint_stall(aero_results, prob_cfg)
%
%   Ensures the stall speed is below acceptable limits for safe operation.
%   For amphibious aircraft, low stall speed is critical for water operations.
%
%   Constraint: V_stall <= V_stall_max  →  g <= 0
%
%   Inputs:
%     aero_results - Struct from compute_aero()
%     prob_cfg     - Problem config struct
%
%   Output:
%     g - Scalar constraint value (g <= 0 means feasible)

    % --- Maximum acceptable stall speed ---
    % For amphibious: lower is better (shorter water run)
    % FAR 23 / CS-23: V_stall <= 61 knots (31.4 m/s) for single-engine
    V_stall_max = 30;  % [m/s] ~ 58 knots

    % --- Compute stall speed ---
    W = prob_cfg.aircraft.MTOW * prob_cfg.atmo.g;
    rho = prob_cfg.atmo.rho_sl;
    S_ref = aero_results.S_ref;
    CL_max = aero_results.CL_max;

    if CL_max <= 0 || S_ref <= 0
        g = 1.0;  % Invalid: constraint violated
        return;
    end

    V_stall = sqrt(2 * W / (rho * S_ref * CL_max));

    % --- Normalized constraint ---
    % g = (V_stall - V_stall_max) / V_stall_max
    g = (V_stall - V_stall_max) / V_stall_max;

end
