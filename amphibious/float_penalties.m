function penalties = float_penalties(params, amphi_cfg)
% FLOAT_PENALTIES  Compute drag and weight penalties from amphibious features.
%   penalties = float_penalties(params, amphi_cfg)
%
%   Calculates the aerodynamic and weight penalties imposed by
%   the amphibious configuration (hull shape, step, spray rails, floats).
%
%   Inputs:
%     params    - Struct with design variable fields
%     amphi_cfg - Amphibious config struct
%
%   Output:
%     penalties - Struct with fields:
%                 .delta_Cd0       - [-] Total parasitic drag increment
%                 .delta_weight_kg - [kg] Additional structural weight
%                 .wetted_area_add - [m^2] Additional wetted area
%                 .weight_breakdown- Struct with itemized weight penalties

    cfg_drag = amphi_cfg.drag;
    cfg_floats = amphi_cfg.floats;
    hull_cfg = amphi_cfg.hull;

    beam = params.hull_beam;
    hull_length = beam * hull_cfg.length_beam_ratio;

    % --- Drag penalties ---
    % Hull form drag (larger cross-section than streamlined fuselage)
    dCd0_hull = cfg_drag.hull_Cd0_increment;

    % Step drag (flow separation at hull step)
    dCd0_step = cfg_drag.step_Cd0_increment;

    % Spray rails
    dCd0_spray_rails = cfg_drag.spray_rail_Cd0;

    % Float/sponson drag (if applicable)
    dCd0_floats = cfg_floats.drag_increment;

    % Total drag increment
    delta_Cd0 = dCd0_hull + dCd0_step + dCd0_spray_rails + dCd0_floats;

    % --- Weight penalties ---
    MTOW = 1500;  % [kg] - will be coupled from prob_cfg in full integration

    % Hull structure weight (heavier than fuselage due to water loads)
    % Water landing loads require reinforcement
    W_hull_reinforce = MTOW * cfg_floats.weight_fraction;

    % Step reinforcement (stress concentration)
    W_step = beam * hull_cfg.step_height_ratio * 50;  % [kg] empirical

    % Spray rails (aluminum/composite strips)
    W_spray_rails = hull_length * 0.5;  % [kg] ~0.5 kg/m

    % Corrosion protection (marine coating, anodizing)
    W_corrosion = MTOW * 0.01;  % ~1% MTOW

    % Bilge/drainage system
    W_bilge = 5;  % [kg] pumps, drains, seals

    % Total weight penalty
    delta_weight = W_hull_reinforce + W_step + W_spray_rails + W_corrosion + W_bilge;

    % Additional wetted area
    wetted_area_add = cfg_floats.wetted_area_add;

    % --- Build output struct ---
    penalties = struct();
    penalties.delta_Cd0       = delta_Cd0;
    penalties.delta_weight_kg = delta_weight;
    penalties.wetted_area_add = wetted_area_add;

    penalties.weight_breakdown = struct();
    penalties.weight_breakdown.hull_reinforce   = W_hull_reinforce;
    penalties.weight_breakdown.step_structure   = W_step;
    penalties.weight_breakdown.spray_rails      = W_spray_rails;
    penalties.weight_breakdown.corrosion_protect = W_corrosion;
    penalties.weight_breakdown.bilge_system     = W_bilge;

    penalties.drag_breakdown = struct();
    penalties.drag_breakdown.hull     = dCd0_hull;
    penalties.drag_breakdown.step     = dCd0_step;
    penalties.drag_breakdown.spray_rails = dCd0_spray_rails;
    penalties.drag_breakdown.floats   = dCd0_floats;

end
