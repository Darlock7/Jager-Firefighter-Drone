%% ============================================================
%  Parasite Drag Estimation (Raymer Drag Build-Up)
%  Components (current): 1) Fuselage, 2) Two Propeller Rods
%
%  CD0 = sum( Cf_i * FF_i * Q_i * S_wet_i ) / S_ref
%
%  AUTHOR: (Team 1)
%  DATE:   (01.30.2026)
%% ============================================================

clear; clc;

%% ============================================================
%  ASSUMPTIONS (EDIT FOR REPORT)
%  1) Parasite drag is estimated using Raymer-style drag build-up:
%       CD0 = Σ(Cf * FF * Q * Swet) / Sref
%  2) Flow is subsonic; compressibility correction included in Cf_turb
%     using (1 + 0.144 M^2)^0.65.
%  3) Skin friction coefficient uses:
%       Laminar:   Cf_lam  = 1.328 / sqrt(Re)
%       Turbulent: Cf_turb = 0.455 / [(log10(Re))^2.58 * (1+0.144M^2)^0.65]
%  4) Mixed laminar/turbulent is approximated by wetted-area weighting:
%       Cf_eff = xLam*Cf_lam + (1-xLam)*Cf_turb
%  5) Interference factor is constant for all components:
%       Q_i = 1.1   (given by instructor)
%  6) Wetted areas are approximated using simple geometry:
%       - Fuselage and rods treated as cylinders: Swet ≈ π d l
%  7) Form factor (FF) for fuselage-like bodies uses:
%       FF = 1 + 60/f^3 + f/400, where f = l/d
%  8) Sea-level standard atmosphere is assumed (rho, mu, a).
%     If operating altitude differs, update rho, mu, a accordingly.
%  9) S_ref is assumed (placeholder) because wing design is not final.
%     When the wing is defined, set S_ref = wing planform area.
% 10) Rods are assumed to have 0% laminar flow (conservative) because
%     they are slender exposed members and likely experience disturbed flow.
%% ============================================================


%% ============================================================
%  USER-EDITABLE INPUTS (CHANGE THESE FIRST)
%% ============================================================

% --- Flight conditions ---
V_mph = 51.43;           % target speed [mph]
rho = 1.225;             % air density [kg/m^3] (sea level)
mu  = 1.81e-5;           % dynamic viscosity [kg/(m*s)] (sea level)
a   = 340.0;             % speed of sound [m/s] (sea level)

% --- Global constant from instructor ---
Q_const = 1.1;

% --- Reference area ---
S_ref = 0.50;            % [m^2] PLACEHOLDER (set to wing planform area later)

% --- Laminar fractions (Raymer Table 12.4 guidance) ---
% Drone closest to "general aviation—smooth molded composites"
xLam_fuselage = 0.25;    % 25% laminar on fuselage (optimistic but plausible if smooth)
xLam_rods     = 0.00;    % 0% laminar on rods (conservative)

% --- Geometry (PLACEHOLDERS) ---
% Fuselage
geom.fuse.l = 1.20;      % length [m]
geom.fuse.d = 0.15;      % diameter [m]

% Rods (2 rods holding propellers)
geom.rods.n = 2;
geom.rods.l = 0.40;      % length [m]
geom.rods.d = 0.03;      % diameter [m]

%% ============================================================
%  END USER INPUTS
%% ============================================================


%% --- Derived quantities ---
V = V_mph * 0.44704;     % m/s
M = V / a;

%% ============================================================
%  COMPONENT SETUP (EDIT HERE TO ADD MORE AREAS)
%  Each component needs:
%   - name
%   - type: "body" (uses fuselage FF) or other types you add later
%   - length scale L for Reynolds number
%   - wetted area Swet
%   - laminar fraction xLam
%   - interference factor Q
%% ============================================================

components = [];

% 1) Fuselage
comp.name = "Fuselage";
comp.type = "body";
comp.L    = geom.fuse.l;                             % Re length scale
comp.Swet = pi * geom.fuse.d * geom.fuse.l;          % cylinder wetted area
comp.f    = geom.fuse.l / geom.fuse.d;               % fineness ratio
comp.xLam = xLam_fuselage;
comp.Q    = Q_const;
components = [components; comp];

% 2) Rods (two cylinders)
comp.name = "Rods (x2)";
comp.type = "body";
comp.L    = geom.rods.l;
comp.Swet = geom.rods.n * pi * geom.rods.d * geom.rods.l;
comp.f    = geom.rods.l / geom.rods.d;
comp.xLam = xLam_rods;
comp.Q    = Q_const;
components = [components; comp];

%% ============================================================
%  CORE CALCULATION
%% ============================================================

CD0_sum = 0;

fprintf("\n=== Parasite Drag Build-Up ===\n");
fprintf("V = %.2f m/s (%.2f mph), M = %.3f\n", V, V_mph, M);
fprintf("S_ref = %.4f m^2, Q = %.2f\n\n", S_ref, Q_const);

for i = 1:numel(components)
    c = components(i);

    % Reynolds number
    Re = rho * V * c.L / mu;

    % Skin friction coefficients
    Cf_lam  = 1.328 / sqrt(Re);
    Cf_turb = 0.455 / ( (log10(Re))^2.58 * (1 + 0.144*M^2)^0.65 );

    % Effective Cf via wetted-area weighting
    Cf = c.xLam * Cf_lam + (1 - c.xLam) * Cf_turb;

    % Form factor
    switch c.type
        case "body"
            FF = 1 + 60/(c.f^3) + c.f/400;   % fuselage/smooth canopy type
        otherwise
            error("Unknown component type: %s", c.type);
    end

    % Component CD0 contribution
    CD0_i = (Cf * FF * c.Q * c.Swet) / S_ref;

    % Accumulate
    CD0_sum = CD0_sum + CD0_i;

    % Print line item
    fprintf("%s\n", c.name);
    fprintf("  Re   = %.3e\n", Re);
    fprintf("  xLam = %.2f  |  Cf_lam = %.5f  Cf_turb = %.5f  Cf_eff = %.5f\n", ...
            c.xLam, Cf_lam, Cf_turb, Cf);
    fprintf("  FF   = %.3f  |  Q = %.2f  |  Swet = %.4f m^2\n", FF, c.Q, c.Swet);
    fprintf("  CD0  = %.6f\n\n", CD0_i);
end

fprintf("-------------------------------------\n");
fprintf("TOTAL CD0 = %.6f\n", CD0_sum);

%% ============================================================
%  OPTIONAL: PLACEHOLDER SECTION FOR ADDING MORE COMPONENTS
%  Copy/paste a block like this above the calculation loop:
%
%  comp.name = "Wing";
%  comp.type = "lifting_surface";   % you would need to implement FF for this
%  comp.L    = c_bar;
%  comp.Swet = Swet_wing;
%  comp.f    = NaN;                 % not used for lifting_surface
%  comp.xLam = xLam_wing;
%  comp.Q    = Q_const;
%  components = [components; comp];
%
%  Then add a "case" in the FF switch for "lifting_surface"
%% ============================================================


%% ============================================================
%  CD0 vs VELOCITY PLOT
%% ============================================================

% Velocity sweep
V_mph_vec = linspace(10, 80, 50);     % mph (edit as needed)
V_vec = V_mph_vec * 0.44704;          % m/s

CD0_vec = zeros(size(V_vec));

for k = 1:length(V_vec)

    V_k = V_vec(k);
    M_k = V_k / a;

    CD0_sum_k = 0;

    for i = 1:numel(components)
        c = components(i);

        % Reynolds number
        Re = rho * V_k * c.L / mu;

        % Skin friction
        Cf_lam  = 1.328 / sqrt(Re);
        Cf_turb = 0.455 / ( (log10(Re))^2.58 * (1 + 0.144*M_k^2)^0.65 );

        Cf = c.xLam * Cf_lam + (1 - c.xLam) * Cf_turb;

        % Form factor
        switch c.type
            case "body"
                FF = 1 + 60/(c.f^3) + c.f/400;
            otherwise
                error("Unknown component type: %s", c.type);
        end

        % Component contribution
        CD0_i = (Cf * FF * c.Q * c.Swet) / S_ref;

        CD0_sum_k = CD0_sum_k + CD0_i;
    end

    CD0_vec(k) = CD0_sum_k;
end

%% --- Plot ---
figure;
plot(V_mph_vec, CD0_vec, 'LineWidth', 2);
grid on;
xlabel('Velocity [mph]');
ylabel('Parasite Drag Coefficient C_{D0}');
title('Parasite Drag Coefficient vs Velocity');