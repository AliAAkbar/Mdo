function f = objective_weight(design_vars, prob_cfg, prop_results, amphi_results)
% OBJECTIVE_WEIGHT  Total aircraft weight objective.
%   f = objective_weight(design_vars, prob_cfg, prop_results, amphi_results)
%
%   Computes estimated total aircraft weight from component buildups.
%   The optimizer minimizes this directly.
%
%   Inputs:
%     design_vars  - [1 x n_vars] Design variable vector
%     prob_cfg     - Problem config struct
%     prop_results - Propulsion analysis results
%     amphi_results- Amphibious analysis results
%
%   Output:
%     f - Total weight estimate [kg] (lower is better)

    % --- Map design variables ---
    var_names = prob_cfg.var_names;
    params = struct();
    for i = 1:numel(var_names)
        params.(var_names{i}) = design_vars(i);
    end

    MTOW = prob_cfg.aircraft.MTOW;

    % =====================================================================
    % WEIGHT BUILDUP
    % =====================================================================

    % 1. Structural weight (wing + fuselage/hull + empennage)
    W_struct = MTOW * prob_cfg.aircraft.empty_weight_frac;

    % Wing weight (Raymer-style empirical)
    S_ref = params.wing_span * (params.wing_chord_root + params.wing_chord_tip) / 2;
    AR = params.wing_span^2 / S_ref;
    taper = params.wing_chord_tip / params.wing_chord_root;
    % Simplified Nicolai/Raymer: W_wing ~ S^0.5 * AR^0.6 * taper^-0.1
    W_wing = 30 * S_ref^0.649 * AR^0.5 * (1 + taper)^0.1;

    % 2. Propulsion system
    W_propulsion = prop_results.mass_total;

    % 3. Battery
    W_battery = prop_results.battery.mass_total;

    % 4. Amphibious penalties
    W_amphi_penalty = amphi_results.penalties.delta_weight_kg;

    % 5. Fixed equipment (avionics, controls, furnishing)
    W_fixed = 80;  % [kg] Simplified fixed systems weight

    % 6. Payload
    W_payload = prob_cfg.mission.payload_mass;

    % =====================================================================
    % TOTAL WEIGHT
    % =====================================================================
    W_total = W_wing + (W_struct - W_wing) + W_propulsion + W_battery + ...
              W_amphi_penalty + W_fixed + W_payload;

    % Note: W_struct includes wing, but we compute wing separately for
    % sensitivity. Use max to avoid double-counting.
    W_total = max(W_total, W_struct + W_propulsion + W_battery + ...
                  W_amphi_penalty + W_payload);

    f = W_total;

end
