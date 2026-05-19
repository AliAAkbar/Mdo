function battery = battery_model(battery_mass, prop_cfg)
% BATTERY_MODEL  Electric battery sizing and energy computation.
%   battery = battery_model(battery_mass, prop_cfg)
%
%   Models a lithium-ion battery pack including pack-level losses,
%   depth of discharge limits, degradation, and reserve requirements.
%
%   Inputs:
%     battery_mass - [kg] Total battery mass (cells + BMS + thermal)
%     prop_cfg     - Propulsion config struct (from propulsion_config.m)
%
%   Output:
%     battery - Struct with fields:
%               .mass_total      - [kg] Total battery system mass
%               .mass_cells      - [kg] Cell mass only
%               .energy_total    - [Wh] Total stored energy (nameplate)
%               .energy_usable   - [Wh] Usable energy after DoD + reserve + degradation
%               .voltage_nominal - [V] Nominal pack voltage
%               .capacity_Ah     - [Ah] Nominal capacity
%               .power_max       - [W] Maximum discharge power (C-rate limited)
%               .specific_energy_pack - [Wh/kg] Pack-level specific energy

    cfg = prop_cfg.battery;

    % --- Cell mass (remove overhead for BMS, cabling, thermal) ---
    mass_cells = battery_mass / cfg.mass_overhead;

    % --- Energy calculations ---
    energy_total = mass_cells * cfg.specific_energy;  % [Wh] Nameplate at cell level
    energy_pack  = energy_total * cfg.pack_efficiency;  % [Wh] Pack-level energy

    % Usable energy: apply DoD, reserve, degradation, and charge efficiency
    energy_usable = energy_pack * cfg.depth_of_discharge * ...
                    (1 - cfg.min_reserve) * ...
                    cfg.degradation_factor * ...
                    cfg.charge_efficiency;

    % --- Electrical parameters ---
    voltage = cfg.voltage_nominal;
    capacity_Ah = energy_pack / voltage;  % [Ah]

    % --- Power limits (assume 3C max discharge for safety) ---
    C_rate_max = 3.0;
    power_max = energy_pack * C_rate_max;  % [W]

    % --- Pack-level specific energy ---
    specific_energy_pack = energy_usable / battery_mass;

    % --- Build output struct ---
    battery = struct();
    battery.mass_total          = battery_mass;
    battery.mass_cells          = mass_cells;
    battery.energy_total        = energy_total;
    battery.energy_usable       = energy_usable;
    battery.voltage_nominal     = voltage;
    battery.capacity_Ah         = capacity_Ah;
    battery.power_max           = power_max;
    battery.specific_energy_pack = specific_energy_pack;

end
