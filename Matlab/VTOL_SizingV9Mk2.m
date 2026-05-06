%% VTOL Sizing Equations
%Compute initial sizing for 55lb > VTOL firefighting drone given user
%inputted mission parameters

%UNITS: This script takes Imperial inputs, computes in SI and outputs Imperial

% UPDATES:
%  V9: includes Vn diagram with wind gusts 
%  Mk2: Mission Profile (B) RECON

clear; clc; close all;

%% ================================
%      Export Calcs as txt file
% =================================
fname = ['Exported_Parameters' '.txt'];
diary(fname);
diary on

%% ============================
%        CONVERSIONS
% =============================
mile_to_m    = 1609.344;                % [m/mile]
mps_to_mph   = 2.2369362920544;         % [mph/(m/s)]
mps_to_kph   = 3.6;                     % [km/h/(m/s)]
lbs_to_kg    = 0.45359237;              % [kg/lb]
lbf_to_N     = 4.4482216152605;         % [N/lbf]
ft_to_m      = 0.3048;                  % [m/ft]
m_to_ft      = 3.2808398950131;         % [ft/m]
mps_to_fps   = 3.2808398950131;         % [ft/s per m/s]     

%% =========================
%          GIVEN
% =========================
g            = 9.80665;    % gravity [m/s^2]
roh_seaLevel = 1.225;                   % air density at sea level [kg/m^3]

%% =========================
%      RESEARCHED VALUES
% (values we need to find on a table or justify by comparison)
% =========================
eta_b           = 0.6429;            % battery->propulsive conversion efficiency [-]
LD              = 6;                 % lift-to-drag ratio in cruise [-]
W_eFracW_0      = 0.763;        % empty-weight fraction 
e_b_J_per_kg    = 5.415e5;    % battery specific energy [J/kg]
AR              = 6;          %Typical Aspect Ratio (AR) for UAV: 8-10
Cl_max_2d       = 1.4;
wingSweep       = 0;
CD0             = 0.040;      % parasite drag coefficient [-]
e               = 0.80;       % Oswald efficiency [-]
CLmax           = 0.9*Cl_max_2d*cos(wingSweep);        % max lift coefficient [-]
eta_prop        = 0.70;   % cruise propulsive efficiency (shaft->thrust power) [-]
eta_rotor       = 0.70;   % rotor system efficiency lump (induced + profile + motor/ESC) [-]


% Disk loading selection (W_0/A_tot)
% Efficient-hover band: 240–320 N/m^2
% Balanced/compact band: 320–480 N/m^2
% High disk-loading band: 480–650 N/m^2
W_0FrackA_tot = 250;        % chosen disk loading [N/m^2]


% Imported from Jager Aero Analysis:
LD_max    = 11.063;
%% =========================
%   MISSION PARAMETERS
% =========================
D_cruise_miles        = 1.5;     % one-way point to point distance to target [miles]
D_cruise_m            = D_cruise_miles * mile_to_m;            % distance [m]
nBall                 = 2;       % number of munitions
TimeOnTarget          = 180;     % total allowed time from take off to 1st! payload drop,[s]
t_offDeck             = 30;      % Deployment time [s] (takeoff + climb + align, etc.)
t_targetAcquisition   = 45;      % payload delivery upper bound [s]
t_cruise              = TimeOnTarget - t_offDeck - t_targetAcquisition;
t_overhead            = 40 * (nBall-1); % Additonall alloted hover power for each munition
t_Recovery            = 38; 
t_inHover             = t_targetAcquisition + t_Recovery + t_overhead;
h_target_ft           = 400;     % Maximum altitude as per FAA [ft]
h_Deploy_ft           = 25;
h_target_m            = h_target_ft * ft_to_m;     % target climb altitude [m]
h_Deploy_m            = h_Deploy_ft * ft_to_m;
n_maneuver            = 1.5;   % [-] load factor for coordinated turn sizing
W_0                   = 55;              % takeoff weight [lb or lbf as used in later conversion]
W0_N                  = W_0 * lbf_to_N; 
m0_kg                 = W0_N / g;                % [kg]
W_p                   = 3.30693393*nBall;% payload weight [lb]
%% ==========================================
%          MISSION PROFILE A (air delivery)
% ===========================================
% Primary (A) Specifies the requirements deploying a fire suppressant
% payload on fire.

% --- Phase timing (seconds) ---
 t0 = 0.0;                          % origin
 t1 = t0 + t_offDeck;               % end climb / off-deck
 t2 = t1 + t_cruise;                % end outbound cruise
 t3 = t2 + t_targetAcquisition;      % end descent and drops first payload
 t4 = t3 + t_overhead;                % additional time spent in payload deployment phase
 t5 = t4 + t_offDeck;
 t6 = t5 + t_cruise;
 t7 = t6 + t_Recovery;

% --- Altitude breakpoints (meters) ---
t_bp = [t0,  t1,         t2,         t3,         t4,         t5,         t6,         t7];
h_bp = [0.0, h_target_m, h_target_m, h_Deploy_m, h_Deploy_m, h_target_m, h_target_m, 0.0];

% --- Build a smooth time history ---
dt = 0.25;                         % time resolution [s]
t_vec = (t0:dt:t7).';

% Piecewise-linear altitude vs time
h_vec = interp1(t_bp, h_bp, t_vec, 'linear');

% Optional: convert to feet for plotting
h_vec_ft = h_vec * m_to_ft;

% --- Plot (t0 at origin) ---
figure('Name','Mission Profile');
plot(t_vec, h_vec_ft, 'LineWidth', 2); grid on;
xlabel('Time, t [s]');
ylabel('Altitude, h [ft]');
title('Mission Profile A (air delivery)');

% --- Mark phase boundaries ---
xline(t1, '--', 'Level Off');
xline(t2, '--', 'Target Acquisition Phase');
xline(t3, '--', 'First Payload Away');
xline(t4, '--', 'Winchester (RTB)');
xline(t5, '--', 'Level Off');
xline(t6, '--', 'Recovery');
xline(t7, '--', 'On Deck');

% --- Print phase summary ---
fprintf('--- Mission Profile Timeline (V2.2) ---\n');
fprintf('t0 (start):                       %7.1f s | h = %6.1f ft\n', t0, 0.0);
fprintf('t1 (level off @ target alt):      %7.1f s | h = %6.1f ft\n', t1, h_target_m*m_to_ft);
fprintf('t2 (start target acquisition):    %7.1f s | h = %6.1f ft\n', t2, h_target_m*m_to_ft);
fprintf('t3 (first payload away @ deploy): %7.1f s | h = %6.1f ft\n', t3, h_Deploy_m*m_to_ft);
fprintf('t4 (winchester / RTB @ deploy):   %7.1f s | h = %6.1f ft\n', t4, h_Deploy_m*m_to_ft);
fprintf('t5 (back to target alt):          %7.1f s | h = %6.1f ft\n', t5, h_target_m*m_to_ft);
fprintf('t6 (recovery phase start):        %7.1f s | h = %6.1f ft\n', t6, h_target_m*m_to_ft);
fprintf('t7 (on deck):                     %7.1f s | h = %6.1f ft\n\n', t7, 0.0);

%% =============================
%     CRUISE SPEED REQUIREMENT
% ==============================
if t_cruise <= 0
    error('No time left for cruise. Reduce Take off, target aquisition, or increase total time budget.');
end

V_mps = D_cruise_m / t_cruise;
V_kph = V_mps * mps_to_kph;
V_mph = V_mps * mps_to_mph;
V_stall_mps = V_mps;
V_recon = V_mph * .8;

fprintf('--- Cruise Speed Requirement ---\n');
fprintf('Total time limit:                  %.1f s\n', TimeOnTarget);
fprintf('Distance to target:                %.3f miles (%.1f m)\n', D_cruise_miles, D_cruise_m);
fprintf('Time to reach cruise:              %.1f s\n', t_offDeck);
fprintf('Time available for cruise:         %.1f s\n', t_cruise);
fprintf('Required cruise speed:             %.3f m/s | %.2f km/h | %.2f mph\n\n', V_mps, V_kph, V_mph);

%% ==================================
%      CRUISE BATTERY FRACTION
%  R = eta_b * (e_b/g) * (L/D) * (Wb/W0)
% ===================================
R_m  = 3 * D_cruise_m;            % range used for sizing [m]
R_km = R_m / 1000;
R_mi = R_m / mile_to_m;

% Solved for battery fraction (as implemented)
W_bFrackW_0Cruise = R_m * g/(eta_b*e_b_J_per_kg) * 1/LD;

fprintf('--- Battery mass fraction WRT Cruise Range ---\n');
fprintf('number of munitions:               %.1f\n', nBall);
fprintf('W_p:                               %.1f lbs\n', W_p);
fprintf('eta_b: %.3f, L/D: %.2f\n', eta_b, LD);
fprintf('e_b:   %.0f J/kg\n', e_b_J_per_kg);
fprintf('Range: %.1f m | %.3f km | %.3f miles\n', R_m, R_km, R_mi);
fprintf('=> Wb/W0 (CRUISE):                  %.3f\n\n', W_bFrackW_0Cruise);

%% =========================
%   HOVER BATTERY FRACTION
% =========================
% Thrust required and available (with margin)
T_req = W0_N;     % [N]
T_sf  = T_req * 1.1;        % safety factor margin

% Total rotor disk area implied by disk loading
A_tot   = W0_N / W_0FrackA_tot;              % [m^2]

% Hover power required (as implemented)
P_hover = (T_sf^(3/2))/((2*roh_seaLevel*A_tot)^(1/2)); % [W]

% Hover battery fraction (as implemented)
W_bFrackW_0Hover = t_inHover * P_hover/( eta_b*e_b_J_per_kg*W_0*lbs_to_kg);

% Total battery fraction
W_bFrackW_0 = W_bFrackW_0Hover + W_bFrackW_0Cruise;

fprintf('--- Battery mass fraction WRT Hover Power Requirements ---\n');
fprintf('Disk loading:                       %.1f N/m^2\n', W_0FrackA_tot);
fprintf('Total rotor disk area:              %.4f m^2\n', A_tot);
fprintf('Thrust required / available:        %.2f N / %.2f N\n', T_req, T_sf);
fprintf('Hover power required:               %.2f W\n', P_hover);
fprintf('Hover time used:                    %.1f s\n', t_inHover);
fprintf('=> Wb/W0 (HOVER):                   %.3f\n\n', W_bFrackW_0Hover);

%% ===================================
%    VERTICAL CLIMB BATTERY FRACTION
%  Uses axial-climb momentum theory to estimate climb power, then BMF
%
%  Mission Inputs:
%    - target altitude (h_target_m)
%    - climb time (t_climb_s)
%
%  Core Equations:
%    v_h = sqrt(T/(2*roh_seaLevel*A_tot))
%    v_i = 0.5*(-V_c + sqrt(V_c^2 + 4*v_h^2))
%    P_ideal_climb = T*(V_c + v_i)
%    BMF_climb = (P_ideal_climb * t_climb_s) / (eta_b2s * e_b_J_per_kg * m0_kg)
% =========================
    
% --- Derived climb rate ---
V_c = h_target_m / t_offDeck;    % climb velocity [m/s]

% --- Axial climb momentum theory ---
v_h = sqrt(T_sf / (2*roh_seaLevel*A_tot));               % hover induced velocity [m/s]
v_i = 0.5 * (-V_c + sqrt(V_c^2 + 4*v_h^2));               % induced velocity in climb [m/s]
P_climb = T_sf * (V_c + v_i);                                   % ideal climb power [W]

% -------------------------
% BATTERY MASS FRACTION (BMF) FOR CLIMB
% -------------------------
W_bFrackW_0Climb = (P_climb * 2 * t_offDeck) / (eta_b * e_b_J_per_kg * m0_kg);

% "Equivalent horizontal speed" over your 1-way range (not vertical climb speed).
% This is only for intuition if you want to compare to dash speed outputs.
V_eq_mps = D_cruise_m / t_offDeck;            % [m/s] using your existing D_m from cruise block
V_eq_mph = V_eq_mps * mps_to_mph;      % [mph]
V_c_mps = h_target_m / t_offDeck;      % climb velocity [m/s]
V_c_fps = V_c_mps * mps_to_fps;        % [ft/s]
V_c_mph = V_c_mps * mps_to_mph;        % [MPH]

fprintf('--- Battery mass fraction WRT Climb Power Requirements ---\n');
fprintf('Target altitude:                   %.1f m | %.1f ft\n', h_target_m, h_target_ft);
fprintf('Climb time:                        %.1f s\n', t_offDeck);
fprintf('Climb rate V_c:                    %.2f m/s | %.2f ft/s | %.2f MPH \n', V_c_mps, V_c_fps, V_c_mph);
fprintf('Climb power (ideal):               %.2f W\n', P_climb);
fprintf('=> Wb/W0 (CLIMB):                  %.4f\n\n', W_bFrackW_0Climb);
W_bFrackW_0 = W_bFrackW_0 + W_bFrackW_0Climb;
fprintf('=> => Total Wb/W0 (Hover + Cruise + Climb): %.4f\n\n', W_bFrackW_0);

%% ========================================
%     AVAILABLE/UNDEFINED MASS FRACTION
% ========================================

massPercentPAYLOAD = W_p/W_0 * 100;
massPercentAIRFRAME = W_eFracW_0 * 100;
massPercentBATTERY = W_bFrackW_0 *100;

MassPercentUndefined = 100 - massPercentBATTERY - massPercentPAYLOAD - massPercentAIRFRAME;

fprintf('--- Final Weight Fractions ---\n');
fprintf('Battery Mass percentage:          %.1f %%\n', massPercentBATTERY);
fprintf('Airframe Mass percentage:         %.1f %%\n', massPercentAIRFRAME);
fprintf('Payload Mass percentage:          %.1f %%\n', massPercentPAYLOAD);
fprintf('=> Available/Undefined Mass percentage: %.1f %%\n\n', MassPercentUndefined);

%% ========================================
%     WING LOADING & SIZING (VTOL)
%   Constraint plot in P/W vs W/S for VTOL UAV:
%     1) Wing-borne cruise (P/W vs W/S)
%     2) Wing-borne maneuver (load-factor turn)  <-- from sizing slides
%     3) Rotor-borne hover (flat line in P/W)
%     4) Rotor-borne vertical climb (flat line in P/W)
%     5) Stall limit (max allowable W/S)
%
%   Sea-level density reference (roh_seaLevel):
%     - Cambridge ISA table: rho_sl = 1.225 kg/m^3
%       https://www-mdp.eng.cam.ac.uk/web/library/enginfo/aerothermal_dvd_only/aero/atmos/atmos.html
%     - ERAU ISA notes: rho0 = 1.225 kg/m^3
%       https://eaglepubs.erau.edu/introductiontoaerospaceflightvehicles/chapter/international-standard-atmosphere-isa/
% ========================================
% Given:
V_turn_mps = V_mps*sqrt(n_maneuver); % [m/s] maneuver speed (default = cruise speed)

% Plot setup:
WS_min = 20;        % [N/m^2]
WS_max = 600;       % [N/m^2]
Npts   = 400;
WS_vec = linspace(WS_min, WS_max, Npts);   % [N/m^2]


% Solving for STALL LIMIT (max allowable W/S):
%   W/S <= 0.5*roh_seaLevel*Vstall^2*CLmax
% -------------------------
WS_stall_max = 0.5 * roh_seaLevel * V_stall_mps^2 * CLmax;   % [N/m^2]

% -------------------------
% COMMON TERMS
% -------------------------
k_ind = 1/(pi*e*AR);   % induced-drag factor in CD = CD0 + k_ind*CL^2

q_cruise = 0.5 * roh_seaLevel * V_mps^2;       % [N/m^2]
q_turn   = 0.5 * roh_seaLevel * V_turn_mps^2;  % [N/m^2]

% -------------------------
% 1) WING-BORNE CRUISE CONSTRAINT (P/W vs W/S)
% Drag polar: CD = CD0 + CL^2/(pi e AR)
% D/W = CD0*q/(W/S) + (W/S)/(q*pi*e*AR)
% P/W = (D/W)*V / eta_prop
% -------------------------
DW_cruise = (CD0 .* q_cruise ./ WS_vec) + (k_ind .* WS_vec ./ q_cruise);  % [-]
PW_cruise = (DW_cruise .* V_mps) ./ eta_prop;                              % [W/N]

% -------------------------
% 2) WING-BORNE MANEUVER CONSTRAINT (P/W vs W/S)
% Slides: T/W >= CD0*q/(W/S) + n^2*(W/S)/(pi*e*AR*q)
% Convert to prop P/W:  P/W = (T/W)*V/eta_prop
% -------------------------
TW_maneuver = (CD0 .* q_turn ./ WS_vec) + ((n_maneuver^2) .* k_ind .* WS_vec ./ q_turn); % [-]
PW_maneuver = (TW_maneuver .* V_turn_mps) ./ eta_prop;                                   % [W/N]

% -------------------------
% 3) ROTOR-BORNE HOVER CONSTRAINT (P/W, independent of W/S)
% For fixed disk loading DL = W/A:
% Ideal induced hover P/W = sqrt(DL/(2*roh_seaLevel))
% Apply rotor system efficiency (eta_rotor)
% -------------------------                                       
PW_hover = P_hover / (eta_rotor * W0_N);       % [W/N]

% -------------------------
% 4) VERTICAL CLIMB CONSTRAINT
% Axial climb momentum theory:
%   
% -------------------------
PW_climb = P_climb / (eta_rotor * W0_N);  % [W/N]

% -------------------------
% ENVELOPE (max of constraints)
% -------------------------
PW_req = max([PW_cruise; PW_maneuver; ...
              PW_hover*ones(size(PW_cruise)); ...
              PW_climb*ones(size(PW_cruise))], [], 1);

% -------------------------
% FEASIBLE REGION (stall limit)
% -------------------------
feasible = WS_vec <= WS_stall_max;

WS_feas     = WS_vec(feasible);
PW_req_feas = PW_req(feasible);

% -------------------------
% "BEST" POINT = MAX W/S (closest to stall boundary)
% -------------------------
WS_best = WS_feas(end);         % largest feasible wing loading
PW_best = PW_req_feas(end);     % envelope value at that W/S

% -------------------------
% BUFFERED DESIGN POINT (buffer LEFT of max W/S)
% -------------------------
buf_WS = 0.025;   % 
buf_PW = 0.025;  % 

WS_design = WS_best * (1 - buf_WS);  % left of stall boundary
WS_design = max(WS_design, WS_min);  % keep inside plot domain

% Required P/W at WS_design (interpolate envelope)
PW_req_design = interp1(WS_vec, PW_req, WS_design, 'linear', 'extrap');

% Add "up" buffer (guaranteed >= envelope at WS_design)
PW_design = PW_req_design * (1 + buf_PW);

% Implied power at W0 (metric)
P_design_W = PW_design * W0_N;  % [W]
P_design_kW = P_design_W / 1000;

% Unit conversions
Nm2_per_lbf_ft2   = 47.88025898;                  % 1 (lbf/ft^2) = 47.88025898 (N/m^2)
WN_to_hp_per_lbf  = (4.4482216152605/745.699872); % (W/N) -> (hp/lbf)

WS_best_imp    = WS_best      / Nm2_per_lbf_ft2;
WS_design_imp  = WS_design    / Nm2_per_lbf_ft2;
WS_stall_imp   = WS_stall_max / Nm2_per_lbf_ft2;

PW_best_imp    = PW_best      * WN_to_hp_per_lbf;
PW_design_imp  = PW_design    * WN_to_hp_per_lbf;


% -------------------------
% PLOT:
% -------------------------
fig = figure('Name','VTOL Constraint Plot','Color','w');
ax  = axes(fig); hold(ax,'on'); box(ax,'on'); grid(ax,'on');

% % Force light-mode
% set(ax, ...
%     'Color','w', ...
%     'XColor','k','YColor','k', ...
%     'GridColor','k', ...
%     'GridAlpha',0.15, ...
%     'MinorGridAlpha',0.08);

% Curves
hCruise   = plot(ax, WS_vec, PW_cruise,   'LineWidth', 2);
hMan      = plot(ax, WS_vec, PW_maneuver, 'LineWidth', 2);
hHover    = yline(ax, PW_hover, 'LineWidth', 2);
hClimb    = yline(ax, PW_climb, 'LineWidth', 2);
hEnv      = plot(ax, WS_vec, PW_req,      'LineWidth', 3);   % thicker envelope
hStall    = xline(ax, WS_stall_max, 'LineWidth', 2);

hBest     = plot(ax, WS_best,   PW_best,   'o', 'MarkerSize', 7, 'LineWidth', 2);
hDesign   = plot(ax, WS_design, PW_design, 's', 'MarkerSize', 8, 'LineWidth', 2);

% color select
hCruise.Color = [0.0000 0.4470 0.7410];   % blue
hMan.Color    = [0.8500 0.3250 0.0980];   % orange
hHover.Color  = [0.4660 0.6740 0.1880];   % green
hClimb.Color  = [0.4940 0.1840 0.5560];   % purple
hEnv.Color    = [1.0000 1.0000 0.0000];   % black (envelope)
hStall.Color  = [0.6350 0.1780 0.0840];   % dark red (stall limit)

hBest.Color   = [0.0000 0.0000 0.0000];   % black marker edge
hDesign.Color = [0.0000 0.0000 0.0000];


% Mark both points: min-envelope and buffered design
plot(ax, WS_best,   PW_best,   'o', 'MarkerSize', 7, 'LineWidth', 2);
plot(ax, WS_design, PW_design, 's', 'MarkerSize', 8, 'LineWidth', 2);

xlabel(ax, 'Wing Loading W/S [N/m^2]');
ylabel(ax, 'Power Loading P/W [W/N]');
title(ax,  'Power to Weight & Wing Loading Constraints');

legend(ax, ...
    'Cruise [P/W]', ...
    sprintf('Maneuver (n=%.2f @ V=%.1f m/s) [P/W]', n_maneuver, V_turn_mps), ...
    'Hover [P/W]', ...
    'Vertical climb [P/W]', ...
    'Envelope: max(all constraints)', ...
    'Stall limit [max W/S]', ...
    'Min-envelope point', ...
    sprintf('Buffered design (+%.0f%% P/W, +%.0f%% W/S)', 100*buf_PW, 100*buf_WS), ...
    'Location','best' ...
);

% -------------------------
% ON-PLOT TEXT:
% -------------------------
txt = sprintf([ ...
    'Buffered design:\n' ...
    'W/S = %.1f N/m^2 | %8.3f lbf/ft^2\n' ...
    'P/W = %.4f W/N | %8.6f hp/lbf' ], ...
    WS_design, WS_design_imp, PW_design, PW_design_imp);

annotation(fig, 'textbox', [0.60 0.15 0.34 0.22], ...
    'String', txt, ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', 'k', ...
    'FontName', 'Consolas', ...
    'FontSize', 10);


% -------------------------
% COMMAND WINDOW PRINT:
% -------------------------


fprintf('\n============================================================\n');
fprintf('VTOL CONSTRAINT PLOT SUMMARY\n');
fprintf('============================================================\n');

fprintf('--- Stall / Feasibility ---\n');
fprintf('WS_stall,max        = %8.2f N/m^2 | %8.3f lbf/ft^2\n\n', WS_stall_max, WS_stall_imp);

fprintf('--- Selected Point (best) ---\n');
fprintf('WS_best             = %8.2f N/m^2 | %8.3f lbf/ft^2\n', WS_best, WS_best_imp);
fprintf('PW_best             = %8.5f W/N   | %8.6f hp/lbf\n\n', PW_best, PW_best_imp);

fprintf('--- Buffered Design Point ---\n');
fprintf('buf_WS              = %8.2f %%  (left)\n', 100*buf_WS);
fprintf('buf_PW              = %8.2f %%  (up)\n', 100*buf_PW);
fprintf('WS_design           = %8.2f N/m^2 | %8.3f lbf/ft^2\n', WS_design, WS_design_imp);
fprintf('PW_design           = %8.5f W/N   | %8.6f hp/lbf\n', PW_design, PW_design_imp);

fprintf('============================================================\n\n');

%% ========================================================
%                     AERO DYNAMICS
%    ================================================

%Calculations
S = (1/WS_design_imp)*W_0;
S_si = S * ft_to_m^2;
b = sqrt(AR*S);
b_si = b * ft_to_m;
c_ft  = S / b;                       % chord [ft]
c_in  = c_ft * 12;                   % chord [in]

%Print:
fprintf('--- Aero Dynamics ---\n');
fprintf('Wingspan (b)         = %8.2f ft  | %8.2f m \n', b, b_si);
fprintf('Wing Area (S)        = %8.2f ft^2| %8.4f m^2 \n', S, S_si);
fprintf('AR                   = %8.2f \n', AR);
fprintf('Chord (C)            = %0.6f ft  | %0.4f in\n', c_ft, c_in);
fprintf('Dynamic Pressure (q) = %8.2f pa  \n\n',q_cruise);

%% ============================================================
%        NACA 4412 -> Point Cloud Export for SolidWorks
% ============================================================

% --- Discretization ---
N_pts = 200;                         % points per surface (upper/lower)

% --- Generate normalized (0..1) airfoil coords ---
% NACA 4412: m=0.04, p=0.4, t=0.12
[xu, yu, xl, yl] = naca4_coords(0.04, 0.4, 0.12, N_pts);

% --- Scale to chord (INCHES) ---
scale   = c_in;                      % [in]
unitTag = "in";

Xu = xu * scale;  Yu = yu * scale;   % [in]
Xl = xl * scale;  Yl = yl * scale;   % [in]

% --- Build XYZ point clouds (z=0 for 2D profile) ---
Z  = zeros(size(Xu));

upper_xyz = [Xu(:), Yu(:), Z(:)];
lower_xyz = [Xl(:), Yl(:), Z(:)];

% Closed loop (TE->LE on upper, then LE->TE on lower)
closed_xyz = [upper_xyz; flipud(lower_xyz(2:end-1,:)); upper_xyz(1,:)];

% --- Export filenames with timestamp ---
ts = datestr(now,'yyyymmdd_HHMMSS');

fn_upper  = sprintf('NACA4412_upper_%s_chord_%0.4fin.txt',  ts, scale);
fn_lower  = sprintf('NACA4412_lower_%s_chord_%0.4fin.txt',  ts, scale);
fn_closed = sprintf('NACA4412_closed_%s_chord_%0.4fin.txt', ts, scale);


%UN-comment to export airfoil!!!
% writematrix(upper_xyz,  fn_upper,  'Delimiter','tab');
% writematrix(lower_xyz,  fn_lower,  'Delimiter','tab');
% writematrix(closed_xyz, fn_closed, 'Delimiter','tab');

% fprintf('--- NACA 4412 EXPORT (INCHES) ---\n');
% fprintf('Chord used: %0.6f ft | %0.4f in\n', c_ft, c_in);
% fprintf('Export units: %s\n', unitTag);
% fprintf('Upper:  %s\n', fn_upper);
% fprintf('Lower:  %s\n', fn_lower);
% fprintf('Closed: %s\n\n', fn_closed);

% --- Quick plot check ---
figure; hold on; grid on; axis equal;
plot(Xu, Yu, 'LineWidth', 1.5);
plot(Xl, Yl, 'LineWidth', 1.5);
xlabel(['x [' char(unitTag) ']']); ylabel(['y [' char(unitTag) ']']);
title('NACA 4412 (Scaled in Inches)');

% ============================================================
% Local function: NACA 4-digit coordinates (cosine spacing)
% ============================================================
function [xu, yu, xl, yl] = naca4_coords(m, p, t, N)
    beta = linspace(0, pi, N);
    x    = (1 - cos(beta)) / 2;  % 0..1

    yt = 5*t*( ...
        0.2969*sqrt(x) ...
      - 0.1260*x ...
      - 0.3516*x.^2 ...
      + 0.2843*x.^3 ...
      - 0.1015*x.^4 );          % closed TE

    yc = zeros(size(x));
    dyc_dx = zeros(size(x));

    idx1 = (x < p);
    idx2 = ~idx1;

    yc(idx1)     = (m/p^2) * (2*p*x(idx1) - x(idx1).^2);
    dyc_dx(idx1) = (2*m/p^2) * (p - x(idx1));

    yc(idx2)     = (m/(1-p)^2) * ((1 - 2*p) + 2*p*x(idx2) - x(idx2).^2);
    dyc_dx(idx2) = (2*m/(1-p)^2) * (p - x(idx2));

    theta = atan(dyc_dx);

    xu = x - yt.*sin(theta);
    yu = yc + yt.*cos(theta);

    xl = x + yt.*sin(theta);
    yl = yc - yt.*cos(theta);
end

%% =========================================================
%   ROTOR DIAMETER FROM DISK LOADING (DL)
%   DL = W / (N * A_disk) ,  A_disk = pi*D^2/4  =>  D = sqrt(4W/(pi*N*DL))
%   Uses your existing: W_0, lbf_to_N, W_0FrackA_tot (DL in N/m^2)
%% =========================================================

% ---- USER INPUT ----
N_rotors = 4;   % number of rotors/props (edit)

% ---- CALCS ----
A_disk = A_tot / N_rotors;          % per-rotor disk area [m^2]
D_m    = sqrt(4*A_disk/pi);         % rotor diameter [m]

% ---- UNIT CONVERSIONS ----
D_ft = D_m * m_to_ft;               % [ft]
D_in = D_ft * 12;                   % [in]

% ---- PRINT ----
fprintf('--- Rotor Disk Sizing from Disk Loading ---\n');
fprintf('Number of rotors (N):              %d\n', N_rotors);
fprintf('Disk loading (DL):                 %.2f N/m^2\n', W_0FrackA_tot);
fprintf('Total disk area (A_tot):           %.4f m^2\n', A_tot);
fprintf('Per-rotor disk area (A_disk):      %.4f m^2\n', A_disk);
fprintf('Rotor diameter (D):                %.4f m | %.3f ft | %.2f in\n\n', D_m, D_ft, D_in);

%% =======================================================
%                  Propulsion Calculations
%          ========================================

PowerReqPerMotor = P_design_W / N_rotors;
minThrustReqPerMotor = T_sf / N_rotors;
minThrustReqPerMotor_Kgf = minThrustReqPerMotor / g;
minThrustReqPerMotor_gf = minThrustReqPerMotor_Kgf * 1000;

RPMLoadingFactor = 0.8;

%from propeller
RPM_max = 5700;

%from battery
Voltage_min = 37.0;               % 10S bat: 37.0 / 42.0 Choose lower


KV_min = RPM_max / (RPMLoadingFactor * Voltage_min);

fprintf('--- Propulsion Calculations ---\n');
fprintf('Power Required per Motor > or = %.4f W \n', PowerReqPerMotor);
fprintf('Thrust Required per Motor > or =  %.4f N | %.4f Kgf | %.4f gf \n', ...
    minThrustReqPerMotor,minThrustReqPerMotor_Kgf,minThrustReqPerMotor_gf);
fprintf('KV recommended:   %.1f RPM/V \n\n', KV_min);




%% ============================================================
% V-TAIL SIZING (Tail Volume Coefficient Method)
% Notes: size equivalent S_h and S_v, then convert to V-tail
% ============================================================

% --- Geometry / moment arms ---
% x_w_qc_from_nose = 2.25;                 % wing quarter-chord location from nose [ft]
% L_tail_qc = 4.25;                         % wing QC -> V-tail root QC (moment arm) [ft]
% x_vtail_qc_from_nose = x_w_qc_from_nose + L_tail_qc;   % [ft]

% V3:

x_w_qc_from_nose = 2.92 + .25*(1.052);    % wing quarter-chord location from nose [ft]
L_tail_qc = 4.25-.92;                         % wing QC -> V-tail root QC (moment arm) [ft]
x_vtail_qc_from_nose = x_w_qc_from_nose + L_tail_qc;   % [ft]


L_h = L_tail_qc;                          % horiz tail arm [ft] (use same arm for first pass)
L_v = L_tail_qc;                          % vert  tail arm [ft]

% --- Wing reference geometry (use your current sizing values) ---
S_w = S;                                  % wing area reference [ft^2]
c_w = c_ft;                               % wing MAC ~ chord [ft] (first-pass)
b_w_actual = 7.14;                        % actual wingspan incl. fuselage [ft] (user-provided)

% --- Choose tail volume coefficients (start bracket; tune after stability check) ---
c_h = 0.50;                               % horiz tail volume coefficient [-]
c_v = 0.04;                               % vert  tail volume coefficient [-]

% --- Equivalent conventional tail areas ---
S_h = (c_h * c_w * S_w) / L_h;            % [ft^2]
S_v = (c_v * b_w_actual * S_w) / L_v;     % [ft^2]

% --- V-tail conversion (per notes) ---
S_total = S_h + S_v;                      % total tail planform area [ft^2]
S_panel = 0.5 * S_total;                  % two identical V-tail panels [ft^2]
gamma_deg = atan( sqrt(S_v / S_h) ) * (180/pi);   % V-tail dihedral [deg]

% --- Optional: pick a tail panel AR to get a starting span/chord ---
AR_tail = 4;                            % panel aspect ratio (typ. 3 to 4 for small UAV tails)
b_panel = sqrt(AR_tail * S_panel);        % panel span [ft] (tip-to-root, in panel plane)
c_bar_panel = S_panel / b_panel;          % mean chord [ft]

% --- Print ---
fprintf('--- V-Tail Sizing (Volume Coefficient Method) ---\n');
fprintf('Wing QC from nose:               %6.2f ft\n', x_w_qc_from_nose);
fprintf('V-tail QC from nose:             %6.2f ft\n', x_vtail_qc_from_nose);
fprintf('Tail arm (QC->QC):                %6.2f ft\n', L_tail_qc);
fprintf('Reference wing: S = %6.2f ft^2 | c = %6.3f ft | b(actual) = %6.2f ft\n', S_w, c_w, b_w_actual);
fprintf('Chosen coeffs: c_h = %.3f, c_v = %.3f\n', c_h, c_v);
fprintf('Eqv horiz tail area S_h:         %6.3f ft^2\n', S_h);
fprintf('Eqv vert  tail area S_v:         %6.3f ft^2\n', S_v);
fprintf('V-tail total area (S_h+S_v):     %6.3f ft^2\n', S_total);
fprintf('Each V-tail panel area:          %6.3f ft^2\n', S_panel);
fprintf('V-tail dihedral gamma:           %6.2f deg\n', gamma_deg);
fprintf('Panel geometry (AR_tail=%.2f):    b_panel = %6.3f ft (%.2f in) | c_bar = %6.3f ft (%.2f in)\n\n', ...
    AR_tail, b_panel, 12*b_panel, c_bar_panel, 12*c_bar_panel);

% ============================================================
% V-TAIL AIRFOIL GEOMETRY FOR SOLIDWORKS LOFT (swept, symmetric)
% Generates 3D point clouds for ROOT and TIP airfoil sections
% Uses NACA 00xx (symmetric)
% ============================================================

% -------------------------
% USER CHOICES (edit these)
% -------------------------
t_c          = 0.10;     % thickness ratio for NACA 00xx (0.10 => NACA 0010)
N_pts        = 300;      % points per surface (upper/lower)


% V-tail panel geometry (you can tie these to your earlier prints)
AR_tail      = 3;      % tail panel aspect ratio used for rough sizing
S_panel_ft2  = S_panel;  % from your V-tail sizing section [ft^2]
b_panel_ft   = sqrt(AR_tail * S_panel_ft2);   % panel span (tip-to-root projected) [ft]

% Chords (choose; keep reasonable for structure/actuators)
c_root_ft    = 1;     % root chord [ft]  (~6.6 in example)
taper        = 0.60;     % c_tip / c_root
c_tip_ft     = taper * c_root_ft;

% Sweep + incidence
sweep_deg    = 30;       % geometric sweep angle of panel LE/25% line [deg]
i_tail_deg   = 0;        % tail incidence angle [deg] (set 0 first pass)

% Location references (your geometry)
x_wing_qc_ft = x_w_qc_from_nose;     % wing quarter-chord from nose [ft]
L_tail_ft    = L_tail_qc;     % wing 1/4c to tail root 1/4c [ft]
x_tail_qc_ft = x_wing_qc_ft + L_tail_ft;  % tail root quarter-chord from nose [ft]

% V-tail dihedral angle from sizing section (already computed earlier)
gamma = gamma_deg;       % [deg]

% -------------------------
% DERIVED 3D PLACEMENT (NO DIHEDRAL)
% -------------------------

y_tip_ft   = b_panel_ft;                       % span direction
x_sweep_ft = y_tip_ft * tand(sweep_deg);       % sweep shift
z_tip_ft   = 0;                                % keep flat (apply gamma in CAD)

% Root/tip quarter-chord points
Pqc_root = [x_tail_qc_ft, 0, 0];
Pqc_tip  = [x_tail_qc_ft + x_sweep_ft, y_tip_ft, z_tip_ft];

% -------------------------
% CLOSED PERIMETER (but do NOT repeat the first point at the end)
% -------------------------
[xu, yu, xl, yl] = naca00xx_coords_closedTE(t_c, N_pts);

% Build perimeter: TE->LE (upper), then LE->TE (lower)
% Remove duplicate LE (xl(1)) and duplicate TE (xl(end)) to prevent loops
x2d = [xu; xl(2:end-1)];
y2d = [yu; yl(2:end-1)];   % thickness axis (local z)


% -------------------------
% BUILD ROOT SECTION IN 3D
% -------------------------
root = place_airfoil_3D(x2d, y2d, c_root_ft, i_tail_deg, Pqc_root);

% -------------------------
% BUILD TIP SECTION IN 3D
% -------------------------
tip  = place_airfoil_3D(x2d, y2d, c_tip_ft,  i_tail_deg, Pqc_tip);

% -------------------------
% EXPORT (XYZ) FOR SOLIDWORKS CURVE THROUGH XYZ POINTS
% -------------------------
% -------------------------
% EXPORT UNITS
% -------------------------
export_units = "in";   % "in" | "mm" | "ft"

switch export_units
    case "in"
        scale = 12.0;        % ft -> in
        units_label = 'in';
    case "mm"
        scale = 304.8;       % ft -> mm
        units_label = 'mm';
    case "ft"
        scale = 1.0;         % ft -> ft
        units_label = 'ft';
    otherwise
        error('export_units must be "in", "mm", or "ft".');
end

root_out = root * scale;
tip_out  = tip  * scale;

fprintf('\n============================================================\n');
fprintf('V-TAIL AIRFOIL EXPORT (Swept Symmetric NACA 00xx)\n');
fprintf('============================================================\n');
fprintf('Airfoil: NACA %04d  (t/c = %.2f)\n', round(100*t_c), t_c);
fprintf('Root chord: %.3f ft | Tip chord: %.3f ft | Taper: %.2f\n', c_root_ft, c_tip_ft, taper);
fprintf('Panel span: %.3f ft | Sweep: %.1f deg | Dihedral(gamma): %.2f deg\n', b_panel_ft, sweep_deg, gamma);
fprintf('Tail QC from nose: %.3f ft | Export units: %s\n', x_tail_qc_ft, units_label);

% Write files (comment these out if you don't want files)
writematrix(root_out, 'Vtail_root_airfoil_XYZ.txt', 'Delimiter', 'tab');
writematrix(tip_out,  'Vtail_tip_airfoil_XYZ.txt',  'Delimiter', 'tab');

fprintf('Wrote: Vtail_root_airfoil_XYZ.txt\n');
fprintf('Wrote: Vtail_tip_airfoil_XYZ.txt\n\n');

% ============================================================
% LOCAL FUNCTIONS
% ============================================================
function [xu,yu,xl,yl] = naca00xx_coords_closedTE(t, N)
    beta = linspace(0, pi, N)';
    x = 0.5*(1 - cos(beta));   % 0..1

    % Closed TE coefficient (-0.1036 instead of -0.1015)
    yt = 5*t*( 0.2969*sqrt(x) ...
             - 0.1260*x ...
             - 0.3516*x.^2 ...
             + 0.2843*x.^3 ...
             - 0.1036*x.^4 );

    % Upper surface TE->LE, lower LE->TE
    xu = flipud(x);  yu = flipud( yt);
    xl = x;          yl = -yt;
end


function pts3 = place_airfoil_3D(x2d, y2d, chord_ft, inc_deg, Pqc_ft)
    % Places a 2D airfoil curve into 3D:
    % - Scales by chord
    % - Rotates about quarter-chord by incidence
    % - Translates so quarter-chord is at Pqc_ft
    %
    % 2D is in local (x along chord, z thickness), and y=0 plane.
    %
    % Output pts3: [X Y Z] in ft

    % Scale
    x = x2d * chord_ft;
    z = y2d * chord_ft;
    y = zeros(size(x));

    % Shift so quarter-chord is at origin
    x = x - 0.25*chord_ft;

    % Incidence rotation about Y-axis (pitch)
    th = deg2rad(inc_deg);
    Xr =  x*cos(th) + z*sin(th);
    Zr = -x*sin(th) + z*cos(th);

    % Translate to quarter-chord location
    pts3 = [Xr + Pqc_ft(1), y + Pqc_ft(2), Zr + Pqc_ft(3)];
end

%% ============================================================
% STATIC STABILITY (FIRST-PASS): Neutral Point ONLY
% Reports x_NP as inches | feet and as %MAC
% Notes:
% - Rectangular wing => MAC = c_w
% - Wing AC assumed at 25% MAC (quarter-chord)
% - For V-tail: uses S_h (equivalent horizontal tail area) as stabilizing area
% ============================================================

% -------------------------
% 1) MAC and MAC leading edge location (from nose datum)
% -------------------------
MAC_ft = c_w;                                              % [ft] rectangular wing => MAC = chord
x_LE_MAC_from_nose_ft = x_w_qc_from_nose - 0.25*MAC_ft;    % [ft] LE of MAC (since QC = LE + 0.25c)

% Wing aerodynamic center (first-pass): ~ quarter-chord
x_ac_w_from_nose_ft = x_w_qc_from_nose;                    % [ft]

% -------------------------
% 2) Tail terms + assumptions
% -------------------------
S_t = S_h;              % [ft^2] equivalent horizontal tail area (V-tail stability component)
l_t = L_tail_qc;        % [ft] moment arm QC->QC (wing AC to tail AC)

eta_t       = 0.95;     % tail efficiency (typical first-pass 0.9–1.0)
deps_dalpha = 0.35;     % downwash gradient dε/dα (typical first-pass 0.3–0.5)

% Lift-curve slopes (per rad), simple finite-wing estimate
a_w = 2*pi*AR      / (AR      + 2);    % wing [1/rad]
a_t = 2*pi*AR_tail / (AR_tail + 2);    % tail [1/rad]

% -------------------------
% 3) Neutral Point estimate
% x_NP/c = x_ac_w/c + eta*(a_t/a_w)*(S_t/S_w)*(1-dε/dα)*(l_t/c)
% -------------------------
xNP_over_c = ( (x_ac_w_from_nose_ft - x_LE_MAC_from_nose_ft)/MAC_ft ) + ...
             ( eta_t * (a_t/a_w) * (S_t/S_w) * (1 - deps_dalpha) * (l_t/MAC_ft) );

x_NP_from_nose_ft = x_LE_MAC_from_nose_ft + xNP_over_c * MAC_ft;   % [ft]
x_NP_percent_MAC  = 100 * xNP_over_c;                              % [% MAC]

% -------------------------
% 4) Print (in | ft)
% -------------------------
fprintf('--- Static Stability: Neutral Point ---\n');
fprintf('MAC:                             %.4f in | %.4f ft\n', 12*MAC_ft, MAC_ft);

fprintf('x_LE_MAC from nose:              %.2f in | %.4f ft\n', ...
        12*x_LE_MAC_from_nose_ft, x_LE_MAC_from_nose_ft);

fprintf('Wing AC (assumed @ 25%% MAC):     %.2f in | %.4f ft\n', ...
        12*x_ac_w_from_nose_ft, x_ac_w_from_nose_ft);

fprintf('Tail arm l_t (QC->QC):           %.2f in | %.4f ft\n', ...
        12*l_t, l_t);

fprintf('Assumptions: eta_t=%.2f, dε/dα=%.2f, a_w=%.3f 1/rad, a_t=%.3f 1/rad\n', ...
        eta_t, deps_dalpha, a_w, a_t);

fprintf('Neutral Point, x_NP:             %.2f in | %.4f ft  |  %.2f %% MAC\n\n', ...
        12*x_NP_from_nose_ft, x_NP_from_nose_ft, x_NP_percent_MAC);

%% ============================================================
%  V-n DIAGRAM (Maneuver Envelope)
%  Uses: V_stall_mps (stall), V_mps (cruise-required speed in your script)
% ============================================================

% --- Load factor limits (edit if your requirement differs) ---
if ~exist('n_pos_limit','var'); n_pos_limit = 3.9; end      % +g limit
if ~exist('n_neg_limit','var'); n_neg_limit = -1.5; end     % -g limit

% --- Reference speeds (m/s) ---
Vs = V_stall_mps;         % stall speed [m/s]
Vc = V_mps;               % "cruise" speed (your required cruise speed) [m/s]

% Maneuver speed where positive stall curve hits +g limit:
Va = Vs * sqrt(n_pos_limit);                               % [m/s]

% Design dive speed (simple sizing default; adjust if you have a better Vd):
Vd = max(1.25*Vc, 1.10*Va);                                % [m/s]

% --- Build envelope ---
N = 600;
V = linspace(0, Vd, N);                                    % [m/s]

% Stall boundaries (assuming symmetric negative CL capability for envelope sketch)
n_stall_pos = (V./Vs).^2;
n_stall_neg = -(V./Vs).^2;

% Clip to limit load factors
n_upper = min(n_stall_pos, n_pos_limit);
n_lower = max(n_stall_neg, n_neg_limit);

% --- Plot (mph for x-axis, per your imperial outputs) ---
V_mph = V * mps_to_mph;

figure('Name','V-n Diagram','Color','w'); hold on; grid on;

% Upper / lower boundaries
h_mUpper = plot(V_mph, n_upper, 'LineWidth', 2, 'Color', 'blue');
h_mLower = plot(V_mph, n_lower, 'LineWidth', 2, 'Color', 'red');

% Limit load factor lines (horizontal)
h_nPos = plot([Va Vd]*mps_to_mph, [n_pos_limit n_pos_limit], 'LineWidth', 2, 'Color', 'magenta');
h_nNeg = plot([Va Vd]*mps_to_mph, [n_neg_limit n_neg_limit], 'LineWidth', 2, 'Color', 'magenta');

% Vertical "dive speed" closure line
h_vdClose = plot([Vd Vd]*mps_to_mph, [n_neg_limit n_pos_limit], 'LineWidth', 2, 'Color', 'green');

% Reference speed markers
xline(Vs*mps_to_mph, '--', sprintf('V_s = V_c = %.2f mph', Vs*mps_to_mph), ...
    'LabelOrientation','horizontal', ...
    'LabelVerticalAlignment','top');
xline(Va*mps_to_mph, '--', sprintf('V_a = %.2f mph', Va*mps_to_mph), ...
    'LabelOrientation','horizontal', ...
    'LabelVerticalAlignment','top');
xline(Vd*mps_to_mph, '--', sprintf('V_d = %.2f mph', Vd*mps_to_mph), ...
    'LabelOrientation','horizontal', ...
    'LabelVerticalAlignment','bottom');

xlabel('Airspeed V [mph]');
ylabel('Load Factor n');
title('V-n Diagram (Maneuver Envelope)');
ylim([min(n_neg_limit, -2.0)-0.2, max(n_pos_limit, 4.0)+0.2]);

% --- Print key values ---
fprintf('\n--- V-n Diagram Speeds ---\n');
fprintf('Vs (stall)     = %8.3f m/s | %8.2f mph\n', Vs, Vs*mps_to_mph);
fprintf('Va (maneuver)  = %8.3f m/s | %8.2f mph   (Va = Vs*sqrt(n_pos_limit))\n', Va, Va*mps_to_mph);
fprintf('Vc (cruise)    = %8.3f m/s | %8.2f mph\n', Vc, Vc*mps_to_mph);
fprintf('Vd (dive)      = %8.3f m/s | %8.2f mph   (default = max(1.25*Vc, 1.10*Va))\n', Vd, Vd*mps_to_mph);
fprintf('n limits       = %+5.2f / %+5.2f\n', n_pos_limit, n_neg_limit);

%% ============================================================
%  V-n DIAGRAM OVERLAY: WIND GUST ENVELOPE (Vertical Gusts)
%  Based on Raymer-style gust model in your notes:
%    Δn = (q * CLα * (U/V)) / (W/S), with U = K*Ude
% ============================================================

% ---- REQUIRED aerodynamic/geo inputs (set if your script doesn't already) ----
% CLalpha: lift-curve slope [per rad]. If you only have per-degree, convert.
if ~exist('CLalpha_per_rad','var')
    % Reasonable default for a finite wing if nothing else is available:
    % (You should replace this with your actual wing CLα if you have it.)
    CLalpha_per_rad = 5.0;   % [1/rad]  (placeholder)
end

% Mean aerodynamic chord [m]
if ~exist('c_bar_m','var')
    if exist('MAC','var') && ~isempty(MAC)
        % If MAC is in INCHES in your script, comment this out and set c_bar_m manually
        c_bar_m = MAC * 0.0254;   % [m]
    else
        c_bar_m = 0.30;           % [m] placeholder
    end
end

% Wing loading [N/m^2] for gust calcs (gust loads depend on W/S)
% Prefer a design wing loading variable if you already compute one.
if ~exist('W_S_gust','var')
    if exist('W_S','var') && ~isempty(W_S)
        % If W_S in your script is already N/m^2, you're good.
        W_S_gust = W_S;                       % [N/m^2]
    elseif exist('W0_lb','var') && exist('S_ft2','var')
        W_S_gust = (W0_lb*4.44822) / (S_ft2*0.092903);   % [N/m^2]
    else
        W_S_gust = 600;                        % [N/m^2] placeholder
    end
end

% Air density and gravity
if ~exist('roh_seaLevel','var'); roh_seaLevel = 1.225; end   % [kg/m^3]
g0 = 9.80665;                                                % [m/s^2]

% ---- Gust velocities ----
% Standard "derived equivalent gust" Ude: 30 ft/s up to Vc, then linear to 15 ft/s at Vd
ftps_to_mps = 0.3048;
Ude_lo = 30 * ftps_to_mps;   % [m/s] :contentReference[oaicite:3]{index=3}
Ude_hi = 15 * ftps_to_mps;   % [m/s] :contentReference[oaicite:4]{index=4}

Ude_V = zeros(size(V));      % V is your existing speed vector [m/s] from the V-n section
for i = 1:numel(V)
    if V(i) <= Vc
        Ude_V(i) = Ude_lo;
    else
        % linear drop from (Vc,30) to (Vd,15)
        Ude_V(i) = Ude_lo + (Ude_hi - Ude_lo) * ( (V(i)-Vc) / max(Vd-Vc, eps) );
    end
end

% ---- Gust alleviation factor K via mass ratio μ ----
mu = (2*W_S_gust) / (roh_seaLevel * g0 * c_bar_m * CLalpha_per_rad);  % :contentReference[oaicite:5]{index=5}
K  = (0.88*mu) / (5.3 + mu);                                          % :contentReference[oaicite:6]{index=6}

U_V = K .* Ude_V;  % effective gust velocity U = K*Ude :contentReference[oaicite:7]{index=7}

% ---- Gust load factor increment Δn ----
% From notes: Δn = q*CLα*(U/V)/(W/S), q=0.5*rho*V^2  => Δn = 0.5*rho*V*U*CLα / (W/S)
Delta_n = 0.5 * roh_seaLevel .* V .* U_V .* CLalpha_per_rad ./ W_S_gust;  % :contentReference[oaicite:8]{index=8}

% Gust lines are applied about 1-g level flight: n = 1 ± Δn :contentReference[oaicite:9]{index=9}
n_gust_up = 1 + Delta_n;
n_gust_dn = 1 - Delta_n;

% ---- Overlay on existing V-n figure ----
% (Assumes you already did: figure(...); hold on; plot(V_mph, n_upper)... etc.)
h_gUp = plot(V_mph, n_gust_up, 'LineWidth', 2, 'Color', 'cyan');
h_gDn = plot(V_mph, n_gust_dn, 'LineWidth', 2, 'Color', 'cyan');

% show the *governing* envelope (max of maneuver/gust upper; min of lower)
n_upper_governing = max(n_upper, n_gust_up);
n_lower_governing = min(n_lower, n_gust_dn);

% ---- Highlight / shade the governing flight envelope ----
x_env = [V_mph, fliplr(V_mph)];
y_env = [n_upper_governing, fliplr(n_lower_governing)];

h_env = fill(x_env, y_env, [0.85 0.92 1.00], ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.35);

% Send the shaded region behind all plotted lines
uistack(h_env, 'bottom');

legend([h_mUpper h_mLower h_nPos h_nNeg h_vdClose h_gUp h_gDn], ...
    {'Positive stall boundary', ...
     'Negative stall boundary', ...
     '+n structural limit', ...
     '-n structural limit', ...
     'V_d boundary', ...
     'Gust upper (1+\Delta n)', ...
     'Gust lower (1-\Delta n)'}, ...
    'Location','northwest');

% ---- Print gust parameters ----
fprintf('\n--- Gust Overlay (Vertical Gust) ---\n');
fprintf('CLalpha used      = %8.3f 1/rad\n', CLalpha_per_rad);
fprintf('c_bar used        = %8.3f m\n', c_bar_m);
fprintf('W/S used          = %8.2f N/m^2\n', W_S_gust);
fprintf('Mass ratio mu     = %8.3f\n', mu);
fprintf('Gust factor K     = %8.3f\n', K);
fprintf('Ude: 30 ft/s -> 15 ft/s from Vc to Vd (linear)\n');  % :contentReference[oaicite:10]{index=10}

diary off

%% ===================================
%      Mission Profile B (RECON)
%   ================================
% For loitering in cruise giving valuable real-time information to
% emergency responders for fire fighting and search and rescue.

% ------------------------------------------------------------
% 1) Battery fraction available for recon cruise/loiter
% ------------------------------------------------------------

% determining range by additional/available battery by mass:
Max_reconBatteryCapacity_FrackW_0 = W_bFrackW_0 + W_p/W_0;

W_bFrackW_0Hover_B = t_Recovery * P_hover / (eta_b * e_b_J_per_kg * W_0 * lbs_to_kg);
W_bFrackW_0Climb_B = (P_climb * t_offDeck) / (eta_b * e_b_J_per_kg * m0_kg);

reconBatteryCapacity_FrackW_0 = Max_reconBatteryCapacity_FrackW_0 ...
    - W_bFrackW_0Hover_B - W_bFrackW_0Climb_B - W_bFrackW_0Cruise;

%  R = (eta_b * e_b / g) * (L/D) * (Wb/W0)
reconRange = (eta_b * e_b_J_per_kg) / g * reconBatteryCapacity_FrackW_0 * LD_max; % [m]
reconRange_mi = reconRange / mile_to_m; % [mi]

% ------------------------------------------------------------
% 2) Recon time (unit-safe)
% ------------------------------------------------------------
% If V_cruise is in mph: use reconRange_mi
% If V_cruise is in m/s: use reconRange (m)

if V_recon > 30  % heuristic: >30 is almost certainly mph for small UAV sizing
    t_recon = (reconRange_mi / V_recon) * 3600;   % [s]
    V_cruise_mph = V_recon;
    V_cruise_ms  = V_recon * mile_to_m / 3600;
else
    t_recon = reconRange / V_recon;               % [s]
    V_cruise_ms  = V_recon;
    V_cruise_mph = V_recon * 3600 / mile_to_m;
end

% ------------------------------------------------------------
% 3) Phase timing (seconds) - 5 events
% ------------------------------------------------------------
t0B = 0.0;
t1B = t0B + t_offDeck;        % end climb / off-deck
t2B = t1B + t_cruise;         % outbound cruise
t3B = t2B + t_recon;          % recon loiter segment (modeled as cruise endurance)
t4B = t3B + t_cruise;         % return cruise
t5B = t4B + t_Recovery;       % recovery/land

% ------------------------------------------------------------
% 4) Altitude breakpoints (meters)
% ------------------------------------------------------------
t_bp_B = [t0B,  t1B,       t2B,       t3B,       t4B,       t5B];
h_bp_B = [0.0,  h_target_m, h_target_m, h_target_m, h_target_m, 0.0];

% --- Time history (match Mission A style) ---
t_vec_B = (t0B:dt:t5B).';
h_vec_B = interp1(t_bp_B, h_bp_B, t_vec_B, 'linear');
h_vec_B_ft = h_vec_B * m_to_ft;

% ------------------------------------------------------------
% 5) Plot 1: Mission B 
% ------------------------------------------------------------

%% ==========================================
% Mission Profile B (broken x-axis plot)
% Skip: 150 s to 2880 s
%% ==========================================

t_skip_start = 150;
t_skip_end   = 2860;

ymin = min(h_vec_B_ft);
ymax = max(h_vec_B_ft);

figure('Name','Mission Profile B (RECON)');

left_pos  = [0.10 0.12 0.38 0.78];
right_pos = [0.55 0.12 0.38 0.78];

%% ================= LEFT AXIS =================
ax1 = axes('Position', left_pos);
plot(ax1, t_vec_B, h_vec_B_ft,'LineWidth',2);
grid(ax1,'on'); hold(ax1,'on');

xlim(ax1,[t0B t_skip_start]);
ylim(ax1,[ymin ymax]);

xlabel(ax1,'Time, t [s]');
ylabel(ax1,'Altitude, h [ft]');
title(ax1,'Mission Profile B (RECON)');

% Events before loiter
if t1B <= t_skip_start
    xline(ax1,t1B,'--','Level Off');
end

% Begin loiter
xline(ax1,t2B,'--','Begin Loiter',...
    'LabelVerticalAlignment','bottom');

%% ================= RIGHT AXIS =================
ax2 = axes('Position', right_pos);
plot(ax2, t_vec_B, h_vec_B_ft,'LineWidth',2);
grid(ax2,'on'); hold(ax2,'on');

xlim(ax2,[t_skip_end t5B]);
ylim(ax2,[ymin ymax]);

xlabel(ax2,'Time, t [s]');
yticklabels(ax2,[]);

% ---- FORCE End Loiter label to appear ----
xline(ax2,t3B,'--','End Loiter',...
    'LabelVerticalAlignment','bottom');

if t4B >= t_skip_end
    xline(ax2,t4B,'--','Recovery');
end

xline(ax2,t5B,'--','On Deck');

%% ================= AXIS BREAK MARKS =================
break_size = 0.015;

p1 = left_pos;
p2 = right_pos;

x1 = p1(1)+p1(3);
y1 = p1(2);
y2 = p1(2)+p1(4);
x2 = p2(1);

annotation('line',[x1-break_size x1+break_size],[y1+0.02 y1],'LineWidth',1.5);
annotation('line',[x2-break_size x2+break_size],[y1+0.02 y1],'LineWidth',1.5);

annotation('line',[x1-break_size x1+break_size],[y2 y2-0.02],'LineWidth',1.5);
annotation('line',[x2-break_size x2+break_size],[y2 y2-0.02],'LineWidth',1.5);

%% ================= SKIP LABEL =================
skip_str = sprintf('loiter compressed\n[%0.0f-%0.0f] s', ...
                   t_skip_start,t_skip_end);

annotation('textbox',[0.46 0.02 0.12 0.06],...
    'String',skip_str,...
    'EdgeColor','none',...
    'HorizontalAlignment','center',...
    'Interpreter','none');

% ------------------------------------------------------------
% 6) Plot 2: Combined Mission A + Mission B
% ------------------------------------------------------------


t_skip_start = 410;
t_skip_end   = 2860;

% y-limits based on BOTH missions so overlay doesn't clip
ymin_AB = min([h_vec_ft(:); h_vec_B_ft(:)]);
ymax_AB = max([h_vec_ft(:); h_vec_B_ft(:)]);

figure('Name','Mission Profile A vs B (Broken Axis)');

left_pos  = [0.10 0.12 0.38 0.78];
right_pos = [0.55 0.12 0.38 0.78];

%% ================= LEFT AXIS =================
ax1 = axes('Position', left_pos);
plot(ax1, t_vec,   h_vec_ft,   'LineWidth',2); grid(ax1,'on'); hold(ax1,'on');
plot(ax1, t_vec_B, h_vec_B_ft, '--', 'LineWidth',2);

xlim(ax1, [min([t0, t0B]), t_skip_start]);
ylim(ax1, [ymin_AB, ymax_AB]);

xlabel(ax1,'Time, t [s]');
ylabel(ax1,'Altitude, h [ft]');
title(ax1,'Mission Profile Comparison');


% --- Mission A markers on LEFT axis (only if in view) ---
xl = xlim(ax1);

if t1 >= xl(1) && t1 <= xl(2), xline(ax1,t1,'--','Level Off'); end
if t2 >= xl(1) && t2 <= xl(2), xline(ax1,t2,'--','Target Acquisition'); end
if t3 >= xl(1) && t3 <= xl(2), xline(ax1,t3,'--','First Payload Away'); end
if t4 >= xl(1) && t4 <= xl(2), xline(ax1,t4,'--','Winchester (RTB)'); end
if t5 >= xl(1) && t5 <= xl(2), xline(ax1,t5,'--','Level Off'); end
if t6 >= xl(1) && t6 <= xl(2), xline(ax1,t6,'--','Recovery'); end
if t7 >= xl(1) && t7 <= xl(2), xline(ax1,t7,'--','On Deck'); end

% --- Mission B markers on LEFT axis (Begin Loiter) ---
if t2B <= t_skip_start
    xline(ax1, t2B, '--', 'Begin Loiter', 'LabelVerticalAlignment','bottom');
end

%% ================= RIGHT AXIS =================
ax2 = axes('Position', right_pos);
plot(ax2, t_vec,   h_vec_ft,   'LineWidth',2); grid(ax2,'on'); hold(ax2,'on');
plot(ax2, t_vec_B, h_vec_B_ft, '--', 'LineWidth',2);

% Right-side time window (use whichever mission ends later)
t_end_right = max(t_vec(end), t_vec_B(end));

xlim(ax2, [t_skip_end, t_end_right]);
ylim(ax2, [ymin_AB, ymax_AB]);

xlabel(ax2,'Time, t [s]');
yticklabels(ax2,[]);

% Legend 
legend(ax1, 'Mission Profile A (air delivery)', ...
            'Mission Profile B (RECON)', ...
            'Location','best');

% --- Mission A markers on RIGHT axis (only if in view) ---
xr = xlim(ax2);

if t1 >= xr(1) && t1 <= xr(2), xline(ax2,t1,'--','Level Off'); end
if t2 >= xr(1) && t2 <= xr(2), xline(ax2,t2,'--','Target Acquisition'); end
if t3 >= xr(1) && t3 <= xr(2), xline(ax2,t3,'--','First Payload Away'); end
if t4 >= xr(1) && t4 <= xr(2), xline(ax2,t4,'--','Winchester (RTB)'); end
if t5 >= xr(1) && t5 <= xr(2), xline(ax2,t5,'--','Level Off'); end
if t6 >= xr(1) && t6 <= xr(2), xline(ax2,t6,'--','Recovery'); end
if t7 >= xr(1) && t7 <= xr(2), xline(ax2,t7,'--','On Deck'); end

% --- Mission B markers on RIGHT axis (FORCE End Loiter) ---
xline(ax2, t3B, '--', 'End Loiter', 'LabelVerticalAlignment','bottom');

%% ================= AXIS BREAK MARKS =================
break_size = 0.015;

p1 = left_pos;  p2 = right_pos;
x1 = p1(1)+p1(3);
y1 = p1(2);
y2 = p1(2)+p1(4);
x2 = p2(1);

annotation('line',[x1-break_size x1+break_size],[y1+0.02 y1],'LineWidth',1.5);
annotation('line',[x2-break_size x2+break_size],[y1+0.02 y1],'LineWidth',1.5);

annotation('line',[x1-break_size x1+break_size],[y2 y2-0.02],'LineWidth',1.5);
annotation('line',[x2-break_size x2+break_size],[y2 y2-0.02],'LineWidth',1.5);

%% ================= SKIP LABEL =================
skip_str = sprintf('loiter compressed\n[%0.0f-%0.0f] s', t_skip_start, t_skip_end);

annotation('textbox',[0.46 0.02 0.12 0.06],...
    'String',skip_str,...
    'EdgeColor','none',...
    'HorizontalAlignment','center',...
    'Interpreter','none');

% ------------------------------------------------------------
% 7) Print Mission B Summary
% ------------------------------------------------------------
fprintf('--- Mission Profile B (Recon/Loiter) ---\n');
fprintf('Recon battery capacity (Wb/W0): %0.4f\n', reconBatteryCapacity_FrackW_0);
fprintf('Recon range: %0.2f mi (%0.1f m)\n', reconRange_mi, reconRange);
fprintf('Loiter speed: %0.2f mph (%0.2f m/s)\n', V_cruise_mph, V_cruise_ms);
fprintf('Recon time:  %0.1f s  (%0.1f min)\n', t_recon, t_recon/60);
fprintf('t0B Start:        %7.1f s\n', t0B);
fprintf('t1B End climb:    %7.1f s\n', t1B);
fprintf('t2B End outbound: %7.1f s\n', t2B);
fprintf('t3B End recon:    %7.1f s\n', t3B);
fprintf('t4B End RTB:      %7.1f s\n', t4B);
fprintf('t5B On deck:      %7.1f s\n', t5B);
fprintf('Total Mission B:  %7.1f s (%0.1f min)\n\n', t5B, t5B/60);