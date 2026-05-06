%% ============================================================
% DRONE_AERO_ANALYSIS_V1.m
% Simple aero check: CL-alpha + drag polar from CAD wetted areas
% Includes operating-point alpha annotation at the chosen flight speed
% ============================================================

clear; clc; close all;

%% -------------------------
% 1) INPUTS (EDIT THESE)
% -------------------------

% --- Units / constants ---
in2_to_ft2 = 1/144;
ft_to_m    = 0.3048; %#ok<NASGU>
slugft3_to_kgm3 = 515.378818; %#ok<NASGU> (not used)

% --- Flight condition (pick a representative analysis point) ---
V_ft_s   = 75.4306667;        % true airspeed [ft/s]
rho      = 0.0023769;         % sea-level density [slug/ft^3]
mu       = 3.737e-7;          % dynamic viscosity [slug/(ft*s)] ~ sea level
q        = 0.5*rho*V_ft_s^2;  % dynamic pressure [lbf/ft^2] in consistent English units

% --- Aircraft weight ---
W_lbf    = 55;                % aircraft weight [lbf] <-- EDIT

% --- Reference geometry (from CAD ideally) ---
Sref_ft2 = 6.64;              % wing reference area Sref [ft^2]
b_ft     = 6.31;              % span [ft]
AR       = b_ft^2 / Sref_ft2; % aspect ratio

% Oswald efficiency (first pass)
e_oswald = 0.80;              % EDIT if you have a better estimate

% Mean aerodynamic chord for Reynolds number
MAC_ft   = 1.052113;          % [ft]

% --- CAD wetted areas (in^2) ---
Swet_fuse_in2 = 1169.18;      % fuselage wetted area
Swet_wing_in2 = 2096.37;      % wing wetted area (both sides)
Swet_tail_in2 = 472.77;       % tail wetted area (both sides)
Swet_misc_in2 = 230.12;       % booms, nacelles, gear, pods, etc.

% Convert to ft^2
Swet_fuse = Swet_fuse_in2*in2_to_ft2;
Swet_wing = Swet_wing_in2*in2_to_ft2;
Swet_tail = Swet_tail_in2*in2_to_ft2;
Swet_misc = Swet_misc_in2*in2_to_ft2;

% --- Form factor inputs (from CAD) ---
L_fuse_ft    = 92.62/12;      % fuselage length [ft]
Dmax_fuse_ft = 7/12;          % fuselage max diameter/width equiv [ft]
tc_wing      = 0.12;          % wing thickness ratio t/c
tc_tail      = 0.10;          % tail thickness ratio t/c

% --- Lift curve model (first pass) ---
CL0              = 0.0;       % CL at alpha=0 deg
CLalpha_per_deg  = 0.10;      % 1/deg
CLmax            = 1.6;       % stall cap
alpha0_deg       = -4.0;      % zero-lift angle [deg]

alpha_deg = (-12:0.25:18)';   % sweep alpha range

%% -------------------------
% 2) SKIN FRICTION (Cf) + CD0 BUILDUP
% -------------------------
% Reynolds number based on MAC
Re = rho*V_ft_s*MAC_ft / mu;

% Turbulent flat plate Cf
Cf = 0.455 / ((log10(Re))^2.58);

% Form factors
FF_wing = 1 + 2.7*tc_wing + 100*(tc_wing^4);
FF_tail = 1 + 2.7*tc_tail + 100*(tc_tail^4);

% Fuselage form factor
if ~isnan(L_fuse_ft) && ~isnan(Dmax_fuse_ft) && Dmax_fuse_ft > 0
    f_fuse  = L_fuse_ft / Dmax_fuse_ft;
    FF_fuse = 1 + 60/(f_fuse^3) + f_fuse/400;
else
    FF_fuse = 1.10;
end

% Interference factors
Q_fuse = 1.00;
Q_wing = 1.00;
Q_tail = 1.00;
Q_misc = 1.00;

% Wetted area total
Swet_vec   = [Swet_fuse, Swet_wing, Swet_tail, Swet_misc];
Swet_total = sum(Swet_vec(~isnan(Swet_vec)));

% CD0 buildup
CD0_fuse = Cf * FF_fuse * Q_fuse * (Swet_fuse / Sref_ft2);

CD0_wing = 0;
if ~isnan(Swet_wing)
    CD0_wing = Cf * FF_wing * Q_wing * (Swet_wing / Sref_ft2);
end

CD0_tail = 0;
if ~isnan(Swet_tail)
    CD0_tail = Cf * FF_tail * Q_tail * (Swet_tail / Sref_ft2);
end

CD0_misc = 0;
if ~isnan(Swet_misc)
    FF_misc  = 1.20;
    CD0_misc = Cf * FF_misc * Q_misc * (Swet_misc / Sref_ft2);
end

CD0 = CD0_fuse + CD0_wing + CD0_tail + CD0_misc;

% Induced drag factor
k = 1/(pi*e_oswald*AR);

%% -------------------------
% 3) CL(alpha) MODEL
% -------------------------
CL_lin = CL0 + CLalpha_per_deg*(alpha_deg - alpha0_deg);

% Optional stall cap
CL = CL_lin;
CL(CL >  CLmax)      = CLmax;
CL(CL < -0.8*CLmax)  = -0.8*CLmax;

%% -------------------------
% 3.5) REQUIRED CL AND ALPHA AT THIS SPEED
% -------------------------
% For steady level flight at V_ft_s
CL_req = W_lbf / (q*Sref_ft2);

% Back out required alpha from the linear lift model
alpha_req_deg = alpha0_deg + (CL_req - CL0)/CLalpha_per_deg;

% Cap displayed CL if beyond model stall cap
CL_req_plot = CL_req;
if CL_req_plot > CLmax
    CL_req_plot = CLmax;
elseif CL_req_plot < -0.8*CLmax
    CL_req_plot = -0.8*CLmax;
end

% Corresponding drag and L/D at that operating point
CD_req = CD0 + k*CL_req_plot^2;
LD_req = CL_req_plot / CD_req;

%% -------------------------
% 4) DRAG POLAR + L/D MAX
% -------------------------
CD = CD0 + k*CL.^2;
LD = CL ./ CD;

[LDmax, idxLD] = max(LD);
CL_LDmax = CL(idxLD);
CD_LDmax = CD(idxLD);

%% -------------------------
% 5) PRINT SUMMARY
% -------------------------
fprintf('=== AERO CHECK (FIRST PASS) ===\n');
fprintf('V           = %.2f ft/s\n', V_ft_s);
fprintf('rho         = %.6f slug/ft^3\n', rho);
fprintf('Re(MAC)     = %.3e\n', Re);
fprintf('Cf          = %.5f\n', Cf);
fprintf('Sref        = %.4f ft^2\n', Sref_ft2);
fprintf('Swet total  = %.4f ft^2 (NaNs ignored)\n', Swet_total);

fprintf('\n-- CD0 buildup --\n');
fprintf('CD0_fuse    = %.5f\n', CD0_fuse);
fprintf('CD0_wing    = %.5f\n', CD0_wing);
fprintf('CD0_tail    = %.5f\n', CD0_tail);
fprintf('CD0_misc    = %.5f\n', CD0_misc);
fprintf('CD0_total   = %.5f\n', CD0);

fprintf('\n-- Induced drag --\n');
fprintf('AR          = %.3f\n', AR);
fprintf('e           = %.3f\n', e_oswald);
fprintf('k           = %.5f\n', k);

fprintf('\n-- Operating point at V = %.2f ft/s --\n', V_ft_s);
fprintf('Weight      = %.2f lbf\n', W_lbf);
fprintf('CL_required = %.4f\n', CL_req);
fprintf('alpha_req   = %.3f deg\n', alpha_req_deg);
fprintf('CD_at_op    = %.5f\n', CD_req);
fprintf('L/D_at_op   = %.3f\n', LD_req);

fprintf('\n-- L/D max --\n');
fprintf('(L/D)max    = %.3f\n', LDmax);
fprintf('at CL       = %.3f\n', CL_LDmax);
fprintf('at CD       = %.5f\n', CD_LDmax);

if CL_req > CLmax
    fprintf('WARNING: Required CL exceeds CLmax -> speed may be below stall for this model.\n');
end

if isnan(Swet_wing_in2) || isnan(Swet_tail_in2)
    fprintf('\nNOTE: Swet_wing_in2 and/or Swet_tail_in2 is NaN -> CD0 is incomplete.\n');
end
if isnan(L_fuse_ft) || isnan(Dmax_fuse_ft)
    fprintf('NOTE: L_fuse_ft and/or Dmax_fuse_ft is NaN -> fuselage FF is a placeholder.\n');
end

%% -------------------------
% 6) PLOTS
% -------------------------

% (A) CL vs alpha
figure('Name','CL vs alpha');
plot(alpha_deg, CL, 'LineWidth', 1.5); hold on;
grid on; box on;
xlabel('\alpha (deg)','Interpreter','tex');
ylabel('C_L','Interpreter','tex');
title('Lift Curve: C_L vs \alpha','Interpreter','tex');

% Mark operating point at this speed
plot(alpha_req_deg, CL_req_plot, 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
text(alpha_req_deg, CL_req_plot, ...
    sprintf('  V = %.2f ft/s\\newline  \\alpha = %.2f deg\\newline  C_L = %.3f', ...
    V_ft_s, alpha_req_deg, CL_req), ...
    'VerticalAlignment','bottom', 'Interpreter','tex');

hold off;

% (B) CL vs CD with LDmax + CD0 + dashed line from origin to LDmax point
figure('Name','CL vs CD polar');
plot(CD, CL, 'LineWidth', 1.5); hold on;
grid on; box on;
xlabel('C_D','Interpreter','tex');
ylabel('C_L','Interpreter','tex');
title('Drag Polar: C_L vs C_D','Interpreter','tex');

% Mark (L/D)max point
plot(CD_LDmax, CL_LDmax, 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
text(CD_LDmax, CL_LDmax, sprintf('  (L/D)_{max}=%.2f', LDmax), ...
    'VerticalAlignment','bottom','Interpreter','tex');

% Mark CD0 on the polar
plot(CD0, 0, 's', 'MarkerSize', 8, 'LineWidth', 1.5);
text(CD0, 0, sprintf('  C_{D0}=%.4f', CD0), ...
    'VerticalAlignment','bottom','Interpreter','tex');

% Dashed line from origin to LDmax point
plot([0, CD_LDmax], [0, CL_LDmax], '--', 'LineWidth', 1.25);

% Annotate slope = L/D
text(0.5*CD_LDmax, 0.5*CL_LDmax, sprintf('  slope = %.2f', LDmax), ...
    'Interpreter','tex');

% Mark operating point at this speed
plot(CD_req, CL_req_plot, '^', 'MarkerSize', 8, 'LineWidth', 1.5);
text(CD_req, CL_req_plot, ...
    sprintf('  V = %.2f ft/s\\newline  \\alpha = %.2f deg', V_ft_s, alpha_req_deg), ...
    'VerticalAlignment','bottom','Interpreter','tex');

hold off;