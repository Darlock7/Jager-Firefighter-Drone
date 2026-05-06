%% ============================================================
%  JAGER_AVL_AUTORUN_CONTROL_SIZING_V8_OFFTIP_SPREADSHEET.m
%
%  UPDATES IN THIS VERSION
%   1) Allows the aileron and ruddervator control surfaces to stop
%      before the physical tip by using separate control-end stations
%   2) Reorganizes command-window output so the final block prints the
%      required deflection for ALL FOUR PHYSICAL SURFACES for each
%      flight mode
%   3) Exports all key inputs, geometry, case results, derivatives,
%      and final required-deflection summaries to an Excel workbook
%      that is overwritten on every execution
%
%  OUTPUT WORKBOOK
%      jager_control_sizing_results.xlsx
% ============================================================

clear; clc; close all;

%% =========================
% 0) FILENAMES / PATHS
% =========================
avlExe      = "avl352.exe";
geomFile    = "jager.geo.avl";
massFile    = "jager.mass";
cmdFile     = "jager_control_commands.txt";
ftFile      = "jager_ft.txt";
hmFile      = "jager_hm.txt";
stFile      = "jager_st.txt";
logFile     = "jager_avl_log.txt";
resultsFile = "jager_control_sizing_results.xlsx";

%% =========================
% 1) GLOBAL CONSTANTS / UNITS
% =========================
g_ft   = 32.174;          % [ft/s^2]
rho    = 0.0023769;       % [slug/ft^3]
Mach   = 0.00;
CDp    = 0.02813;

%% =========================
% 2) AIRCRAFT WEIGHT / CG / INERTIA
% =========================
W_lbf  = 55;
m_slug = W_lbf / g_ft;

Xcg_in = 39.1;            % [in]
Ycg_ft = 0.0;             % [ft]
Zcg_ft = -1.74/12;        % [ft]
Xcg_ft = Xcg_in/12;

% SolidWorks principal inertias [lbm*ft^2]
Ixx_sw =  75.7671;
Iyy_sw = 119.6108;
Izz_sw = 191.5794;

convI = 1/32.174;
Ixx = Ixx_sw * convI;
Iyy = Iyy_sw * convI;
Izz = Izz_sw * convI;

Ixy = 0;
Ixz = 0;
Iyz = 0;

%% =========================
% 3) AERODYNAMIC REFERENCE GEOMETRY
% =========================
Sref = 6.64;      % [ft^2]
Cref = 1.052;     % [ft]
Bref = 6.31;      % [ft]
b2   = Bref/2;

Xref = Xcg_ft;
Yref = 0.0;
Zref = Zcg_ft;

%% =========================
% 4) CAD DATUM LOCATIONS
% =========================
XwingLE_in = 35.0;
XwingLE    = XwingLE_in/12;   % [ft]

%% =========================
% 5) FLIGHT CONDITION
% =========================
V_ftps    = 50;
alpha_deg = 8.25;
beta_deg  = 0.0;

qbar = 0.5 * rho * V_ftps^2;

%% =========================
% 6) WING GEOMETRY + AILERON
% =========================
c_w = Cref;

% Set eta_a1 < 1.0 to stop the aileron before the physical tip
eta_a0 = 0.6;     % control inboard start
eta_a1 = 0.86;     % control outboard end
eta_aw = 1.00;     % physical wing tip fraction

y_a0  = eta_a0*b2;
y_a1  = eta_a1*b2;
y_tip = eta_aw*b2;

wing_airfoil_type = "NACA";
wing_naca         = "4412";

Xhinge_ail     = 0.85;
aileron_gain   = 1.0;
aileron_SgnDup = -1.0;

%% =========================
% 7) V-TAIL GEOMETRY + CONTROL MIXES
% =========================
S_panel      = 0.809;         % [ft^2] each panel area
AR_tail      = 2.8;           % [-]
taper_vt     = 0.4;           % c_tip / c_root
gamma_deg    = 36.38;         % [deg]
LambdaLE_deg = 45.0;          % [deg]
XtailQC      = 80.3636/12;    % [ft]

b_panel   = sqrt(AR_tail*S_panel);
c_root_vt = 2*S_panel/(b_panel*(1+taper_vt));
c_tip_vt  = taper_vt*c_root_vt;

XtailLE_root = XtailQC - 0.25*c_root_vt;

% Set eta_rv1 < 1.0 to stop the ruddervator before the physical tip
eta_rv0 = 0.20;   % control inboard start
eta_rv1 = .60;   % control outboard end

vtail_airfoil_type = "NACA";
vtail_naca         = "0010";

Xhinge_rv = 0.80;

pitchv_gain = 1.0;
yawv_gain   = 1.0;

%% =========================
% 8) CONTROL TEST SETUP
% =========================
controlMenu_ail    = "D1";
controlName_ail    = "aileron";

controlMenu_pitchv = "D2";
controlName_pitchv = "pitchv";

controlMenu_yawv   = "D3";
controlName_yawv   = "yawv";

delta0 =  0.0;
deltaP = +4.0;
deltaM = -4.0;

t_rate_eval = 0.50;   % [s]

target_roll_degps  = 25.0;
target_pitch_degps =  6.0;
target_yaw_degps   = 10.0;

%% =========================
% 9) PHYSICAL SURFACE LABELS / MIX MAPS
% =========================
surfaceNames = {'Right Aileron','Left Aileron','Right Ruddervator','Left Ruddervator'};

mix_roll  = [ +1, -1,  0,  0];
mix_pitch = [  0,  0, +1, -1];
mix_yaw   = [  0,  0, +1, +1];

%% =========================
% 10) DERIVED CONTROL GEOMETRY FOR PLOTTING / PRINTING
% =========================
x_hinge_ail = XwingLE + Xhinge_ail*c_w;

ail_span  = y_a1 - y_a0;
ail_chord = (1 - Xhinge_ail)*c_w;

% V-tail stations:
% eta0 = root
% eta1 = control start
% eta2 = control end
% eta3 = physical tip
eta0 = 0.0;
eta1 = eta_rv0;
eta2 = eta_rv1;
eta3 = 1.0;

[x0, y0p, z0, c0] = tail_station_from_eta(eta0, XtailLE_root, LambdaLE_deg, b_panel, gamma_deg, c_root_vt, c_tip_vt);
[x1, y1p, z1, c1] = tail_station_from_eta(eta1, XtailLE_root, LambdaLE_deg, b_panel, gamma_deg, c_root_vt, c_tip_vt);
[x2, y2p, z2, c2] = tail_station_from_eta(eta2, XtailLE_root, LambdaLE_deg, b_panel, gamma_deg, c_root_vt, c_tip_vt);
[x3, y3p, z3, c3] = tail_station_from_eta(eta3, XtailLE_root, LambdaLE_deg, b_panel, gamma_deg, c_root_vt, c_tip_vt);

x_hinge0 = x1 + Xhinge_rv*c1;
x_hinge2 = x2 + Xhinge_rv*c2;

rv_span_top   = sqrt((x2-x1)^2 + (y2p-y1p)^2);
rv_span_front = sqrt((y2p-y1p)^2 + (z2-z1)^2);
rv_chord_root = (1 - Xhinge_rv)*c1;
rv_chord_tip  = (1 - Xhinge_rv)*c2;

ft2in = 12.0;
ail_span_in      = ail_span * ft2in;
ail_chord_in     = ail_chord * ft2in;
ail_tip_gap_in   = (y_tip - y_a1) * ft2in;
rv_span_top_in   = rv_span_top * ft2in;
rv_span_front_in = rv_span_front * ft2in;
rv_chord_root_in = rv_chord_root * ft2in;
rv_chord_tip_in  = rv_chord_tip * ft2in;

%% =========================
% 11) QUICK CONTROL GEOMETRY VISUALIZATION
% =========================
figure('Name','JAGER Control Geometry','Color','w');

subplot(1,2,1); hold on; axis equal; grid on;
title('Top View');
xlabel('X [ft]');
ylabel('Y [ft]');

% Wing
xw = [XwingLE, XwingLE + c_w, XwingLE + c_w, XwingLE];
yw = [0,       0,            y_tip,         y_tip];
patch(xw, yw, [0.85 0.85 0.85], 'EdgeColor','k', 'LineWidth',1.2);
patch(xw, -yw, [0.85 0.85 0.85], 'EdgeColor','k', 'LineWidth',1.2);

% Aileron
xa = [x_hinge_ail, XwingLE + c_w, XwingLE + c_w, x_hinge_ail];
ya = [y_a0,        y_a0,          y_a1,         y_a1];
patch(xa, ya, [0.2 0.6 1.0], 'FaceAlpha',0.45, 'EdgeColor','b', 'LineWidth',1.2);
patch(xa, -ya, [0.2 0.6 1.0], 'FaceAlpha',0.45, 'EdgeColor','b', 'LineWidth',1.2);

plot([x_hinge_ail x_hinge_ail], [ y_a0  y_a1], 'b--', 'LineWidth',1.2);
plot([x_hinge_ail x_hinge_ail], [-y_a0 -y_a1], 'b--', 'LineWidth',1.2);

% Whole V-tail panels
x_vr = [x0, x0 + c0, x3 + c3, x3];
y_vr = [y0p, y0p,    y3p,     y3p];
patch(x_vr, y_vr, [0.80 0.90 0.80], 'EdgeColor','k', 'LineWidth',1.2);

x_vl = [x0, x0 + c0, x3 + c3, x3];
y_vl = [-y0p, -y0p,  -y3p,    -y3p];
patch(x_vl, y_vl, [0.80 0.90 0.80], 'EdgeColor','k', 'LineWidth',1.2);

% Ruddervators
xcr = [x_hinge0, x1 + c1, x2 + c2, x_hinge2];
ycr = [y1p,      y1p,     y2p,     y2p];
patch(xcr, ycr, [1.0 0.5 0.2], 'FaceAlpha',0.45, ...
    'EdgeColor',[0.85 0.33 0.1], 'LineWidth',1.2);

xcl = [x_hinge0, x1 + c1, x2 + c2, x_hinge2];
ycl = [-y1p,     -y1p,    -y2p,    -y2p];
patch(xcl, ycl, [1.0 0.5 0.2], 'FaceAlpha',0.45, ...
    'EdgeColor',[0.85 0.33 0.1], 'LineWidth',1.2);

plot([x_hinge0 x_hinge2], [ y1p  y2p], '--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.2);
plot([x_hinge0 x_hinge2], [-y1p -y2p], '--', 'Color',[0.85 0.33 0.1], 'LineWidth',1.2);

text(XwingLE + 0.5*c_w, 0.5*(y_a0+y_a1), ...
    sprintf('Aileron span = %.2f in', ail_span_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge_ail + XwingLE + c_w), y_a1 + 0.08, ...
    sprintf('Aileron chord = %.2f in', ail_chord_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x1+x2), 0.5*(y1p+y2p)+0.08, ...
    sprintf('Ruddervator span = %.2f in', rv_span_top_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge0 + x1 + c1), y1p - 0.10, ...
    sprintf('Root ctrl chord = %.2f in', rv_chord_root_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(x_hinge2 + x2 + c2), y2p + 0.10, ...
    sprintf('Tip ctrl chord = %.2f in', rv_chord_tip_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

legend({'Wing','Wing','Aileron','Aileron','Aileron hinge','Aileron hinge', ...
        'V-tail','V-tail','Ruddervator','Ruddervator'}, ...
        'Location','bestoutside');

subplot(1,2,2); hold on; axis equal; grid on;
title('Front View');
xlabel('Y [ft]');
ylabel('Z [ft]');

plot([-y_tip y_tip],[0 0],'k-','LineWidth',2);
plot([ y_a0  y_a1],[0 0],'b-','LineWidth',6);
plot([-y_a0 -y_a1],[0 0],'b-','LineWidth',6);
plot([ y0p  y3p],[z0 z3],'k-','LineWidth',2);
plot([-y0p -y3p],[z0 z3],'k-','LineWidth',2);
plot([ y1p  y2p],[z1 z2],'-','Color',[0.85 0.33 0.1],'LineWidth',6);
plot([-y1p -y2p],[z1 z2],'-','Color',[0.85 0.33 0.1],'LineWidth',6);

text(0.5*(y_a0+y_a1), 0.08, sprintf('%.2f in', ail_span_in), ...
    'Color','b', 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

text(0.5*(y1p+y2p), 0.5*(z1+z2)+0.08, sprintf('%.2f in', rv_span_front_in), ...
    'Color',[0.85 0.33 0.1], 'FontWeight','bold', ...
    'HorizontalAlignment','center', 'BackgroundColor','w');

legend({'Wing span','Aileron span','Aileron span','V-tail R','V-tail L', ...
        'Ruddervator R','Ruddervator L'}, ...
        'Location','bestoutside');

%% =========================
% 12) CONTROL GEOMETRY SUMMARY
% =========================
fprintf('\n================ CONTROL GEOMETRY SUMMARY ================\n');

fprintf('Aileron:\n');
fprintf('  Inboard start y = %.3f in\n', 12*y_a0);
fprintf('  Outboard end y  = %.3f in\n', 12*y_a1);
fprintf('  Physical tip y  = %.3f in\n', 12*y_tip);
fprintf('  Tip gap         = %.3f in\n', ail_tip_gap_in);
fprintf('  Span            = %.3f in\n', ail_span_in);
fprintf('  Hinge x         = %.3f in\n', 12*x_hinge_ail);
fprintf('  Control chord   = %.3f in\n', ail_chord_in);

fprintf('\nRuddervator:\n');
fprintf('  Inboard start eta      = %.4f\n', eta_rv0);
fprintf('  Outboard end eta       = %.4f\n', eta_rv1);
fprintf('  Start point (x,y,z)    = (%.3f, %.3f, %.3f) in\n', 12*x1, 12*y1p, 12*z1);
fprintf('  End point   (x,y,z)    = (%.3f, %.3f, %.3f) in\n', 12*x2, 12*y2p, 12*z2);
fprintf('  Physical tip (x,y,z)   = (%.3f, %.3f, %.3f) in\n', 12*x3, 12*y3p, 12*z3);
fprintf('  Span along panel       = %.3f in\n', rv_span_top_in);
fprintf('  Front-view span        = %.3f in\n', rv_span_front_in);
fprintf('  Root hinge x           = %.3f in\n', 12*x_hinge0);
fprintf('  Tip hinge x            = %.3f in\n', 12*x_hinge2);
fprintf('  Root ctrl chord        = %.3f in\n', rv_chord_root_in);
fprintf('  Tip ctrl chord         = %.3f in\n', rv_chord_tip_in);

%% ============================================================
% WRITE 1) GEOMETRY FILE
% ============================================================
fid = fopen(geomFile,'w');
assert(fid>0, "Could not write geometry file.");

fprintf(fid,"JAGER Geometry V8 off-tip control support (generated by MATLAB)\n");
fprintf(fid,"%.4f\n", Mach);
fprintf(fid,"0 0 0.0\n");
fprintf(fid,"%.4f %.4f %.4f\n", Sref, Cref, Bref);
fprintf(fid,"%.4f %.4f %.4f\n", Xref, Yref, Zref);
fprintf(fid,"%.6f\n\n", CDp);

% -------------------------
% Wing with off-tip aileron
% -------------------------
fprintf(fid,"SURFACE\nWing\n");
fprintf(fid,"10 1.0 24 1.0\n");
fprintf(fid,"YDUPLICATE\n0.0\n\n");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f 0.0000 0.0000 %.4f 0.0000\n", XwingLE, c_w);
write_airfoil(fid, wing_airfoil_type, wing_naca, "");

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_a0, c_w);
write_airfoil(fid, wing_airfoil_type, wing_naca, "");
fprintf(fid,"CONTROL\n");
fprintf(fid,"aileron %.3f %.3f 0 0 0 %.1f\n\n", ...
    aileron_gain, Xhinge_ail, aileron_SgnDup);

fprintf(fid,"SECTION\n");
fprintf(fid,"%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_a1, c_w);
write_airfoil(fid, wing_airfoil_type, wing_naca, "");
fprintf(fid,"CONTROL\n");
fprintf(fid,"aileron %.3f %.3f 0 0 0 %.1f\n\n", ...
    aileron_gain, Xhinge_ail, aileron_SgnDup);

if y_a1 < y_tip - 1e-9
    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_tip, c_w);
    write_airfoil(fid, wing_airfoil_type, wing_naca, "");
end

% -------------------------
% V-tail surfaces with off-tip ruddervator support
% -------------------------
write_vtail_surface(fid, 'Vtail_R', ...
    +y0p, +z0, c0, x0, ...
    +y1p, +z1, c1, x1, ...
    +y2p, +z2, c2, x2, ...
    +y3p, +z3, c3, x3, ...
    vtail_airfoil_type, vtail_naca, Xhinge_rv, +pitchv_gain, +yawv_gain);

write_vtail_surface(fid, 'Vtail_L', ...
    -y0p, +z0, c0, x0, ...
    -y1p, +z1, c1, x1, ...
    -y2p, +z2, c2, x2, ...
    -y3p, +z3, c3, x3, ...
    vtail_airfoil_type, vtail_naca, Xhinge_rv, -pitchv_gain, +yawv_gain);

fclose(fid);

%% ============================================================
% WRITE 2) MASS FILE
% ============================================================
fid = fopen(massFile,'w');
assert(fid>0, "Could not write mass file.");

fprintf(fid,"# Jager mass file (generated by MATLAB)\n");
fprintf(fid,"# mass  x  y  z  Ixx  Iyy  Izz  Ixz  Ixy  Iyz\n");
fprintf(fid,"# mass units: slug\n");
fprintf(fid,"# length units: ft\n");
fprintf(fid,"# inertia units: slug*ft^2\n");
fprintf(fid,"%.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f\n", ...
    m_slug, Xcg_ft, Ycg_ft, Zcg_ft, Ixx, Iyy, Izz, Ixz, Ixy, Iyz);

fclose(fid);

%% =========================
% 13) PRINT MODEL SUMMARY
% =========================
fprintf("\n================ MODEL SUMMARY ================\n");
fprintf("CG     = (%.4f, %.4f, %.4f) ft\n", Xcg_ft, Ycg_ft, Zcg_ft);
fprintf("Sref   = %.4f ft^2\n", Sref);
fprintf("Cref   = %.4f ft\n", Cref);
fprintf("Bref   = %.4f ft\n", Bref);
fprintf("Alpha  = %.2f deg\n", alpha_deg);
fprintf("Beta   = %.2f deg\n", beta_deg);
fprintf("V      = %.2f ft/s\n", V_ftps);
fprintf("qbar   = %.4f psf\n", qbar);

%% =========================
% 14) RUN AILERON CASES
% =========================
fprintf("\n================ AILERON TEST ================\n");
fprintf("Control tested = %s\n", controlMenu_ail);

out0_ail = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_ail, delta0);

outP_ail = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_ail, deltaP);

outM_ail = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_ail, deltaM);

res_ail = report_control_results(out0_ail, outP_ail, outM_ail, controlName_ail, ...
    qbar, Sref, Bref, Ixx, "roll", target_roll_degps, t_rate_eval, "ROLL MODE / AILERON", ...
    V_ftps, surfaceNames, mix_roll);

%% =========================
% 15) RUN V-TAIL PITCH MIX CASES
% =========================
fprintf("\n================ V-TAIL PITCH MIX TEST ================\n");
fprintf("Control tested = %s\n", controlMenu_pitchv);

out0_pitchv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_pitchv, delta0);

outP_pitchv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_pitchv, deltaP);

outM_pitchv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_pitchv, deltaM);

res_pitch = report_control_results(out0_pitchv, outP_pitchv, outM_pitchv, controlName_pitchv, ...
    qbar, Sref, Cref, Iyy, "pitch", target_pitch_degps, t_rate_eval, "PITCH MODE / V-TAIL MIX", ...
    V_ftps, surfaceNames, mix_pitch);

%% =========================
% 16) RUN V-TAIL YAW MIX CASES
% =========================
fprintf("\n================ V-TAIL YAW MIX TEST ================\n");
fprintf("Control tested = %s\n", controlMenu_yawv);

out0_yawv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_yawv, delta0);

outP_yawv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_yawv, deltaP);

outM_yawv = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu_yawv, deltaM);

res_yaw = report_control_results(out0_yawv, outP_yawv, outM_yawv, controlName_yawv, ...
    qbar, Sref, Bref, Izz, "yaw", target_yaw_degps, t_rate_eval, "YAW MODE / V-TAIL MIX", ...
    V_ftps, surfaceNames, mix_yaw);

%% =========================
% 17) FINAL SUMMARY BLOCK (BOTTOM OF COMMAND WINDOW)
% =========================
results = [res_ail, res_pitch, res_yaw];
modeSummaryTbl = build_mode_summary_table(results);
derivTbl       = build_derivatives_table(results);
ftTbl          = [make_ft_case_table(res_ail); make_ft_case_table(res_pitch); make_ft_case_table(res_yaw)];
stTbl          = build_st_table(results);
hmTbl          = [make_hm_case_table(res_ail); make_hm_case_table(res_pitch); make_hm_case_table(res_yaw)];

print_final_surface_requirement_summary(modeSummaryTbl);

%% =========================
% 18) SPREADSHEET EXPORT (OVERWRITES EVERY RUN)
% =========================
inputsTbl = table( ...
    {'avlExe';'geomFile';'massFile';'resultsFile';'Mach';'CDp';'rho_slug_ft3';'W_lbf';'m_slug'; ...
     'Xcg_in';'Ycg_ft';'Zcg_ft';'Sref_ft2';'Cref_ft';'Bref_ft';'V_ftps';'alpha_deg';'beta_deg'; ...
     'deltaP_deg';'deltaM_deg';'t_rate_eval_s';'target_roll_degps';'target_pitch_degps';'target_yaw_degps'}, ...
    {char(avlExe);char(geomFile);char(massFile);char(resultsFile);Mach;CDp;rho;W_lbf;m_slug; ...
     Xcg_in;Ycg_ft;Zcg_ft;Sref;Cref;Bref;V_ftps;alpha_deg;beta_deg;deltaP;deltaM;t_rate_eval; ...
     target_roll_degps;target_pitch_degps;target_yaw_degps}, ...
    'VariableNames', {'Parameter','Value'});

geometryTbl = table( ...
    {'eta_a0';'eta_a1';'wing_tip_eta';'aileron_span_in';'aileron_chord_in';'aileron_tip_gap_in'; ...
     'eta_rv0';'eta_rv1';'rv_span_top_in';'rv_span_front_in';'rv_chord_root_in';'rv_chord_tip_in'}, ...
    {eta_a0;eta_a1;eta_aw;ail_span_in;ail_chord_in;ail_tip_gap_in;eta_rv0;eta_rv1; ...
     rv_span_top_in;rv_span_front_in;rv_chord_root_in;rv_chord_tip_in}, ...
    'VariableNames', {'Parameter','Value'});

export_results_to_excel(resultsFile, inputsTbl, geometryTbl, modeSummaryTbl, derivTbl, ftTbl, stTbl, hmTbl);

fprintf('\nSpreadsheet written to: %s\n', resultsFile);
fprintf('Done.\n');

%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function out = run_one_case(avlExe, geomFile, massFile, cmdFile, ftFile, hmFile, stFile, logFile, ...
    alpha_deg, beta_deg, controlMenu, controlValue)

    safeDelete(cmdFile);
    safeDelete(logFile);

    touch_empty(ftFile);
    touch_empty(hmFile);
    touch_empty(stFile);

    fid = fopen(cmdFile,'w');
    assert(fid>0, "Could not write AVL command file.");

    fprintf(fid,"LOAD %s\n", geomFile);
    fprintf(fid,"MASS %s\n", massFile);
    fprintf(fid,"OPER\n");
    fprintf(fid,"A A %.6f\n", alpha_deg);
    fprintf(fid,"B B %.6f\n", beta_deg);
    fprintf(fid,"%s %s %.6f\n", controlMenu, controlMenu, controlValue);
    fprintf(fid,"X\n");

    fprintf(fid,"FT\n");
    fprintf(fid,"%s\n", ftFile);
    fprintf(fid,"O\n");

    fprintf(fid,"HM\n");
    fprintf(fid,"%s\n", hmFile);
    fprintf(fid,"O\n");

    fprintf(fid,"ST\n");
    fprintf(fid,"%s\n", stFile);
    fprintf(fid,"O\n");

    fprintf(fid,"\n");
    fprintf(fid,"QUIT\n");
    fclose(fid);

    syscmd = sprintf('"%s" < "%s" > "%s"', avlExe, cmdFile, logFile);
    [status, cmdout] = system(syscmd);

    fprintf("\n------------------------------------------------------------\n");
    fprintf("AVL run for %s = %.3f deg | exit status = %d\n", controlMenu, controlValue, status);
    fprintf("------------------------------------------------------------\n");
    fprintf("%s\n", cmdout);

    if ~isfile(ftFile)
        error("FT output file was not created. Check AVL command flow.");
    end

    if status ~= 0
        warning("AVL returned nonzero exit status, but FT file was created. Continuing.");
    end

    out = struct();
    out.FT = parseFTfile(ftFile);

    if isfile(hmFile)
        out.HM = parseHMfile(hmFile);
    else
        out.HM = struct('name',{},'Chinge',{},'deflection_deg',{},'moment_Nm',{});
    end

    if isfile(stFile)
        out.ST = parseSTfile(stFile);
    else
        out.ST = struct('Cl_pbar',NaN,'Cm_qbar',NaN,'Cn_rbar',NaN);
    end
end

function result = report_control_results(out0, outP, outM, controlName, ...
    qbar, Sref, Lref, Iaxis, axisType, target_rate_degps, t_rate_eval, label, V_ftps, surfaceNames, surfaceMix)

    fprintf('\n================ %s BASELINE FT ================\n', label);
    printFT(out0.FT);

    fprintf('\n================ %s POSITIVE FT ================\n', label);
    printFT(outP.FT);

    fprintf('\n================ %s NEGATIVE FT ================\n', label);
    printFT(outM.FT);

    fprintf('\n================ %s BASELINE HM ================\n', label);
    printHM(out0.HM);

    fprintf('\n================ %s POSITIVE HM ================\n', label);
    printHM(outP.HM);

    fprintf('\n================ %s NEGATIVE HM ================\n', label);
    printHM(outM.HM);

    if isfield(out0,'ST')
        fprintf('\n================ %s BASELINE ST PARSED ================\n', label);
        fprintf('Cl_pbar = %+10.6f\n', out0.ST.Cl_pbar);
        fprintf('Cm_qbar = %+10.6f\n', out0.ST.Cm_qbar);
        fprintf('Cn_rbar = %+10.6f\n', out0.ST.Cn_rbar);
    end

    deltaPlusEcho  = get_control_value(outP.FT.controls, controlName);
    deltaMinusEcho = get_control_value(outM.FT.controls, controlName);

    fprintf('\n========== %s ACTUAL ECHOED DEFLECTIONS ==========\n', label);
    fprintf('delta+ = %+8.4f deg\n', deltaPlusEcho);
    fprintf('delta- = %+8.4f deg\n', deltaMinusEcho);

    if isnan(deltaPlusEcho) || isnan(deltaMinusEcho)
        error("Could not find echoed control '%s' in FT output.", controlName);
    end

    den = deltaPlusEcho - deltaMinusEcho;
    if abs(den) < 1e-12
        error('%s deflection denominator is zero.', label);
    end

    dCl_dDelta = (outP.FT.Cl - outM.FT.Cl) / den;
    dCm_dDelta = (outP.FT.Cm - outM.FT.Cm) / den;
    dCn_dDelta = (outP.FT.Cn - outM.FT.Cn) / den;
    dCY_dDelta = (outP.FT.CY - outM.FT.CY) / den;
    dCL_dDelta = (outP.FT.CL - outM.FT.CL) / den;
    dCD_dDelta = (outP.FT.CD - outM.FT.CD) / den;

    fprintf('\n========== %s FINITE-DIFFERENCE DERIVATIVES ==========\n', label);
    fprintf('dCl/dDelta = %+10.6f per deg\n', dCl_dDelta);
    fprintf('dCm/dDelta = %+10.6f per deg\n', dCm_dDelta);
    fprintf('dCn/dDelta = %+10.6f per deg\n', dCn_dDelta);
    fprintf('dCY/dDelta = %+10.6f per deg\n', dCY_dDelta);
    fprintf('dCL/dDelta = %+10.6f per deg\n', dCL_dDelta);
    fprintf('dCD/dDelta = %+10.6f per deg\n', dCD_dDelta);

    switch lower(string(axisType))
        case "roll"
            coeff_per_deg  = dCl_dDelta;
            coeff_name     = "Cl";
            moment_name    = "L";
            damp_coeff_bar = out0.ST.Cl_pbar;
            rate_name      = "p";
        case "pitch"
            coeff_per_deg  = dCm_dDelta;
            coeff_name     = "Cm";
            moment_name    = "M";
            damp_coeff_bar = out0.ST.Cm_qbar;
            rate_name      = "q";
        case "yaw"
            coeff_per_deg  = dCn_dDelta;
            coeff_name     = "Cn";
            moment_name    = "N";
            damp_coeff_bar = out0.ST.Cn_rbar;
            rate_name      = "r";
        otherwise
            error("Unsupported axisType: %s", string(axisType));
    end

    Mdelta_per_deg = coeff_per_deg * qbar * Sref * Lref;   % [lbf*ft/deg]
    Momega = qbar * Sref * Lref * damp_coeff_bar * (Lref / (2*V_ftps));  % [lbf*ft/(rad/s)]

    fprintf('\n========== %s DAMPED BODY-RATE MODEL ==========\n', label);
    fprintf('%s per deg control            = %+10.6f per deg\n', coeff_name, coeff_per_deg);
    fprintf('%s_delta per deg              = %+10.6f lbf*ft/deg\n', moment_name, Mdelta_per_deg);
    fprintf('%s damping derivative (bar)   = %+10.6f\n', coeff_name, damp_coeff_bar);
    fprintf('d%s/d%s                      = %+10.6f lbf*ft/(rad/s)\n', moment_name, rate_name, Momega);

    method = 'damped';
    tau = NaN;
    omega_ss_per_deg_deg = NaN;

    if isnan(Momega) || abs(Momega) < 1e-12
        fprintf('Damping derivative missing or near zero.\n');
        fprintf('Falling back to undamped estimate.\n');
        method = 'undamped_fallback';

        angacc_per_deg_rad = Mdelta_per_deg / Iaxis;
        angacc_per_deg_deg = angacc_per_deg_rad * 180/pi;
        rate_per_deg = angacc_per_deg_deg * t_rate_eval;

        if abs(rate_per_deg) > 1e-12
            delta_req_deg = target_rate_degps / abs(rate_per_deg);
        else
            delta_req_deg = Inf;
        end

        fprintf('Angular accel per deg         = %+10.6f rad/s^2/deg\n', angacc_per_deg_rad);
        fprintf('Angular accel per deg         = %+10.6f deg/s^2/deg\n', angacc_per_deg_deg);
        fprintf('Angular rate per deg @ %.2f s = %+10.6f deg/s/deg\n', t_rate_eval, rate_per_deg);
        fprintf('Required deflection for %.2f deg/s = %10.6f deg\n', target_rate_degps, delta_req_deg);
    else
        tau = -Iaxis / Momega;
        omega_ss_per_deg_rad = -Mdelta_per_deg / Momega;
        omega_ss_per_deg_deg = omega_ss_per_deg_rad * 180/pi;

        if tau <= 0 || ~isfinite(tau)
            fprintf('Computed time constant is nonphysical (tau = %.6f s).\n', tau);
            fprintf('Check parsed damping derivative sign/value.\n');
            fprintf('Falling back to undamped estimate.\n');
            method = 'undamped_fallback';

            angacc_per_deg_rad = Mdelta_per_deg / Iaxis;
            angacc_per_deg_deg = angacc_per_deg_rad * 180/pi;
            rate_per_deg = angacc_per_deg_deg * t_rate_eval;

            if abs(rate_per_deg) > 1e-12
                delta_req_deg = target_rate_degps / abs(rate_per_deg);
            else
                delta_req_deg = Inf;
            end

            fprintf('Angular accel per deg         = %+10.6f rad/s^2/deg\n', angacc_per_deg_rad);
            fprintf('Angular accel per deg         = %+10.6f deg/s^2/deg\n', angacc_per_deg_deg);
            fprintf('Angular rate per deg @ %.2f s = %+10.6f deg/s/deg\n', t_rate_eval, rate_per_deg);
            fprintf('Required deflection for %.2f deg/s = %10.6f deg\n', target_rate_degps, delta_req_deg);
        else
            rate_per_deg = omega_ss_per_deg_deg * (1 - exp(-t_rate_eval/tau));

            if abs(rate_per_deg) > 1e-12
                delta_req_deg = target_rate_degps / abs(rate_per_deg);
            else
                delta_req_deg = Inf;
            end

            delta_cmd_deg = 10.0;
            rate_cmd_degps = rate_per_deg * delta_cmd_deg;

            fprintf('Axis type                     = %s\n', axisType);
            fprintf('Target rate                   = %.3f deg/s\n', target_rate_degps);
            fprintf('Rate evaluation time          = %.3f s\n', t_rate_eval);
            fprintf('Time constant tau             = %+10.6f s\n', tau);
            fprintf('Steady-state %s per deg       = %+10.6f deg/s/deg\n', rate_name, omega_ss_per_deg_deg);
            fprintf('%s @ %.2f s per deg           = %+10.6f deg/s/deg\n', rate_name, t_rate_eval, rate_per_deg);

            if isfinite(delta_req_deg)
                fprintf('Required deflection for %.2f deg/s = %10.6f deg\n', ...
                    target_rate_degps, delta_req_deg);
            else
                fprintf('Required deflection for %.2f deg/s = INF\n', target_rate_degps);
            end

            fprintf('\n========== %s RESPONSE AT 10 DEG COMMAND ==========\n', label);
            fprintf('Using delta_cmd               = %.2f deg\n', delta_cmd_deg);
            fprintf('%s @ %.2f s                   = %+10.6f deg/s\n', rate_name, t_rate_eval, rate_cmd_degps);
            fprintf('%s_ss                         = %+10.6f deg/s\n', rate_name, omega_ss_per_deg_deg * delta_cmd_deg);
        end
    end

    fprintf('\n========== %s QUICK INTERPRETATION ==========\n', label);
    fprintf('Roll effectiveness magnitude : |dCl/dDelta| = %.6f per deg\n', abs(dCl_dDelta));
    fprintf('Pitch effectiveness magnitude: |dCm/dDelta| = %.6f per deg\n', abs(dCm_dDelta));
    fprintf('Yaw effectiveness magnitude  : |dCn/dDelta| = %.6f per deg\n', abs(dCn_dDelta));
    fprintf('Side-force coupling magnitude: |dCY/dDelta| = %.6f per deg\n', abs(dCY_dDelta));

    result = struct();
    result.label = char(label);
    result.axisType = char(axisType);
    result.controlName = char(controlName);
    result.target_rate_degps = target_rate_degps;
    result.t_rate_eval_s = t_rate_eval;
    result.deltaPlusEcho_deg = deltaPlusEcho;
    result.deltaMinusEcho_deg = deltaMinusEcho;
    result.dCl_dDelta = dCl_dDelta;
    result.dCm_dDelta = dCm_dDelta;
    result.dCn_dDelta = dCn_dDelta;
    result.dCY_dDelta = dCY_dDelta;
    result.dCL_dDelta = dCL_dDelta;
    result.dCD_dDelta = dCD_dDelta;
    result.coeff_per_deg = coeff_per_deg;
    result.coeff_name = char(coeff_name);
    result.moment_name = char(moment_name);
    result.rate_name = char(rate_name);
    result.damp_coeff_bar = damp_coeff_bar;
    result.Mdelta_per_deg = Mdelta_per_deg;
    result.Momega = Momega;
    result.tau_s = tau;
    result.omega_ss_per_deg_degps = omega_ss_per_deg_deg;
    result.rate_per_deg_degps = rate_per_deg;
    result.delta_req_control_deg = delta_req_deg;
    result.method = method;
    result.surfaceNames = surfaceNames;
    result.surfaceMix = surfaceMix;
    result.surfaceDeflections_deg = delta_req_deg * surfaceMix;
    result.out0 = out0;
    result.outP = outP;
    result.outM = outM;
end

function FT = parseFTfile(ftFile)
    txt = fileread(ftFile);

    FT = struct();
    FT.CL = grabScalar(txt, 'CLtot');
    FT.CD = grabScalar(txt, 'CDtot');
    FT.CY = grabScalar(txt, 'CYtot');
    FT.Cl = grabScalar(txt, 'Cltot');
    FT.Cm = grabScalar(txt, 'Cmtot');
    FT.Cn = grabScalar(txt, 'Cntot');
    FT.controls = parseControlEchoes(txt);
end

function ST = parseSTfile(stFile)
    txt = fileread(stFile);

    ST = struct();
    ST.Cl_pbar = NaN;
    ST.Cm_qbar = NaN;
    ST.Cn_rbar = NaN;

    lines = regexp(txt, '\r\n|\n|\r', 'split');

    for i = 1:numel(lines)
        line = strtrim(lines{i});

        if isnan(ST.Cl_pbar)
            v = parse_st_line_value(line, {'Clp','Cl_p','Cl''','Cl(pb/2V)','Cl pb/2V'});
            if ~isnan(v), ST.Cl_pbar = v; end
        end

        if isnan(ST.Cm_qbar)
            v = parse_st_line_value(line, {'Cmq','Cm_q','Cm''','Cm(qc/2V)','Cm qc/2V'});
            if ~isnan(v), ST.Cm_qbar = v; end
        end

        if isnan(ST.Cn_rbar)
            v = parse_st_line_value(line, {'Cnr','Cn_r','Cn''','Cn(rb/2V)','Cn rb/2V'});
            if ~isnan(v), ST.Cn_rbar = v; end
        end
    end

    if isnan(ST.Cl_pbar)
        ST.Cl_pbar = find_coeff_wrt_rate(txt, 'Cl', 'pb/2V');
    end
    if isnan(ST.Cm_qbar)
        ST.Cm_qbar = find_coeff_wrt_rate(txt, 'Cm', 'qc/2V');
    end
    if isnan(ST.Cn_rbar)
        ST.Cn_rbar = find_coeff_wrt_rate(txt, 'Cn', 'rb/2V');
    end
end

function v = parse_st_line_value(line, keys)
    v = NaN;
    for k = 1:numel(keys)
        key = regexptranslate('escape', keys{k});
        tok = regexp(line, [key '\s*=\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)'], 'tokens', 'once');
        if ~isempty(tok)
            v = str2double(tok{1});
            return;
        end
    end
end

function v = find_coeff_wrt_rate(txt, coeffName, rateName)
    v = NaN;
    lines = regexp(txt, '\r\n|\n|\r', 'split');

    for i = 1:numel(lines)
        line = strtrim(lines{i});
        if contains(line, coeffName) && contains(line, rateName)
            nums = regexp(line, '([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)', 'match');
            if ~isempty(nums)
                v = str2double(nums{end});
                return;
            end
        end
    end
end

function HM = parseHMfile(hmFile)
    txt = fileread(hmFile);
    lines = regexp(txt, '\r\n|\n|\r', 'split');

    HM = struct('name',{},'Chinge',{},'deflection_deg',{},'moment_Nm',{});

    for i = 1:numel(lines)
        line = strtrim(lines{i});

        tok = regexp(line, ...
            '^([A-Za-z0-9_\-]+)\s+([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)\s+([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)\s+([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)$', ...
            'tokens', 'once');

        if ~isempty(tok)
            s = struct();
            s.name           = string(tok{1});
            s.Chinge         = str2double(tok{2});
            s.deflection_deg = str2double(tok{3});
            s.moment_Nm      = str2double(tok{4});
            HM(end+1) = s; %#ok<AGROW>
        end
    end
end

function x = grabScalar(txt, label)
    pat = [label '\s*=\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)'];
    tok = regexp(txt, pat, 'tokens', 'once');
    if isempty(tok)
        error('Could not find %s in FT file.', label);
    end
    x = str2double(tok{1});
end

function controls = parseControlEchoes(txt)
    controls = struct('name',{},'value_deg',{});

    lines = regexp(txt, '\r\n|\n|\r', 'split');
    skipNames = ["CLtot","CDtot","CYtot","Cltot","Cmtot","Cntot", ...
                 "CXtot","CZtot","CDvis","CDind","CLff","CDff","CYff", ...
                 "Alpha","Beta","Mach"];

    for i = 1:numel(lines)
        line = strtrim(lines{i});

        tok = regexp(line, ...
            '^([A-Za-z0-9_\-]+)\s*=\s*([-+]?\d*\.?\d+(?:[Ee][-+]?\d+)?)$', ...
            'tokens', 'once');

        if ~isempty(tok)
            nm = string(tok{1});
            if ~ismember(nm, skipNames)
                s = struct();
                s.name = nm;
                s.value_deg = str2double(tok{2});
                controls(end+1) = s; %#ok<AGROW>
            end
        end
    end
end

function val = get_control_value(ctrls, targetName)
    val = NaN;
    for k = 1:numel(ctrls)
        if string(ctrls(k).name) == string(targetName)
            val = ctrls(k).value_deg;
            return;
        end
    end
end

function printFT(FT)
    fprintf('CL = %+10.6f\n', FT.CL);
    fprintf('CD = %+10.6f\n', FT.CD);
    fprintf('CY = %+10.6f\n', FT.CY);
    fprintf('Cl = %+10.6f\n', FT.Cl);
    fprintf('Cm = %+10.6f\n', FT.Cm);
    fprintf('Cn = %+10.6f\n', FT.Cn);

    if ~isempty(FT.controls)
        fprintf('Echoed controls:\n');
        for k = 1:numel(FT.controls)
            fprintf('   %-12s = %+8.4f deg\n', FT.controls(k).name, FT.controls(k).value_deg);
        end
    end
end

function printHM(HM)
    if isempty(HM)
        fprintf('No HM data parsed.\n');
        return;
    end

    for k = 1:numel(HM)
        fprintf('   %-12s | Chinge = %+10.6e | defl = %+8.4f deg | M = %+10.6f N-m\n', ...
            HM(k).name, HM(k).Chinge, HM(k).deflection_deg, HM(k).moment_Nm);
    end
end

function tbl = build_mode_summary_table(results)
    n = numel(results);
    Mode = cell(n,1);
    Axis = cell(n,1);
    Method = cell(n,1);
    TargetRate_degps = zeros(n,1);
    ControlRequired_deg = zeros(n,1);
    RightAileron_deg = zeros(n,1);
    LeftAileron_deg = zeros(n,1);
    RightRuddervator_deg = zeros(n,1);
    LeftRuddervator_deg = zeros(n,1);

    for i = 1:n
        Mode{i} = results(i).label;
        Axis{i} = results(i).axisType;
        Method{i} = results(i).method;
        TargetRate_degps(i) = results(i).target_rate_degps;
        ControlRequired_deg(i) = results(i).delta_req_control_deg;
        RightAileron_deg(i) = results(i).surfaceDeflections_deg(1);
        LeftAileron_deg(i) = results(i).surfaceDeflections_deg(2);
        RightRuddervator_deg(i) = results(i).surfaceDeflections_deg(3);
        LeftRuddervator_deg(i) = results(i).surfaceDeflections_deg(4);
    end

    tbl = table(Mode, Axis, Method, TargetRate_degps, ControlRequired_deg, ...
        RightAileron_deg, LeftAileron_deg, RightRuddervator_deg, LeftRuddervator_deg);
end

function tbl = build_derivatives_table(results)
    n = numel(results);
    Mode = cell(n,1);
    Axis = cell(n,1);
    Method = cell(n,1);
    dCl_dDelta = zeros(n,1);
    dCm_dDelta = zeros(n,1);
    dCn_dDelta = zeros(n,1);
    dCY_dDelta = zeros(n,1);
    dCL_dDelta = zeros(n,1);
    dCD_dDelta = zeros(n,1);
    DampingCoeffBar = zeros(n,1);
    Mdelta_per_deg = zeros(n,1);
    Momega = zeros(n,1);
    Tau_s = zeros(n,1);
    RatePerDeg_degps = zeros(n,1);
    ControlRequired_deg = zeros(n,1);

    for i = 1:n
        Mode{i} = results(i).label;
        Axis{i} = results(i).axisType;
        Method{i} = results(i).method;
        dCl_dDelta(i) = results(i).dCl_dDelta;
        dCm_dDelta(i) = results(i).dCm_dDelta;
        dCn_dDelta(i) = results(i).dCn_dDelta;
        dCY_dDelta(i) = results(i).dCY_dDelta;
        dCL_dDelta(i) = results(i).dCL_dDelta;
        dCD_dDelta(i) = results(i).dCD_dDelta;
        DampingCoeffBar(i) = results(i).damp_coeff_bar;
        Mdelta_per_deg(i) = results(i).Mdelta_per_deg;
        Momega(i) = results(i).Momega;
        Tau_s(i) = results(i).tau_s;
        RatePerDeg_degps(i) = results(i).rate_per_deg_degps;
        ControlRequired_deg(i) = results(i).delta_req_control_deg;
    end

    tbl = table(Mode, Axis, Method, dCl_dDelta, dCm_dDelta, dCn_dDelta, dCY_dDelta, ...
        dCL_dDelta, dCD_dDelta, DampingCoeffBar, Mdelta_per_deg, Momega, Tau_s, ...
        RatePerDeg_degps, ControlRequired_deg);
end

function tbl = make_ft_case_table(result)
    Mode = repmat({result.label}, 3, 1);
    Axis = repmat({result.axisType}, 3, 1);
    Case = {'baseline';'positive';'negative'};
    ControlEcho_deg = [ ...
        get_control_value(result.out0.FT.controls, result.controlName); ...
        get_control_value(result.outP.FT.controls, result.controlName); ...
        get_control_value(result.outM.FT.controls, result.controlName)];
    CL = [result.out0.FT.CL; result.outP.FT.CL; result.outM.FT.CL];
    CD = [result.out0.FT.CD; result.outP.FT.CD; result.outM.FT.CD];
    CY = [result.out0.FT.CY; result.outP.FT.CY; result.outM.FT.CY];
    Cl = [result.out0.FT.Cl; result.outP.FT.Cl; result.outM.FT.Cl];
    Cm = [result.out0.FT.Cm; result.outP.FT.Cm; result.outM.FT.Cm];
    Cn = [result.out0.FT.Cn; result.outP.FT.Cn; result.outM.FT.Cn];

    tbl = table(Mode, Axis, Case, ControlEcho_deg, CL, CD, CY, Cl, Cm, Cn);
end

function tbl = build_st_table(results)
    n = numel(results);
    Mode = cell(n,1);
    Axis = cell(n,1);
    Cl_pbar = zeros(n,1);
    Cm_qbar = zeros(n,1);
    Cn_rbar = zeros(n,1);

    for i = 1:n
        Mode{i} = results(i).label;
        Axis{i} = results(i).axisType;
        Cl_pbar(i) = results(i).out0.ST.Cl_pbar;
        Cm_qbar(i) = results(i).out0.ST.Cm_qbar;
        Cn_rbar(i) = results(i).out0.ST.Cn_rbar;
    end

    tbl = table(Mode, Axis, Cl_pbar, Cm_qbar, Cn_rbar);
end

function tbl = make_hm_case_table(result)
    tbl = empty_hm_table();
    tbl = [tbl; hm_block(result.label, result.axisType, 'baseline', result.out0.HM)]; %#ok<AGROW>
    tbl = [tbl; hm_block(result.label, result.axisType, 'positive', result.outP.HM)]; %#ok<AGROW>
    tbl = [tbl; hm_block(result.label, result.axisType, 'negative', result.outM.HM)]; %#ok<AGROW>
end

function tbl = hm_block(modeLabel, axisType, caseLabel, HM)
    tbl = empty_hm_table();
    if isempty(HM)
        return;
    end

    n = numel(HM);
    Mode = repmat({modeLabel}, n, 1);
    Axis = repmat({axisType}, n, 1);
    Case = repmat({caseLabel}, n, 1);
    Control = cell(n,1);
    Chinge = zeros(n,1);
    Deflection_deg = zeros(n,1);
    Moment_Nm = zeros(n,1);

    for i = 1:n
        Control{i} = char(HM(i).name);
        Chinge(i) = HM(i).Chinge;
        Deflection_deg(i) = HM(i).deflection_deg;
        Moment_Nm(i) = HM(i).moment_Nm;
    end

    tbl = table(Mode, Axis, Case, Control, Chinge, Deflection_deg, Moment_Nm);
end

function tbl = empty_hm_table()
    tbl = table(cell(0,1), cell(0,1), cell(0,1), cell(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
        'VariableNames', {'Mode','Axis','Case','Control','Chinge','Deflection_deg','Moment_Nm'});
end

function print_final_surface_requirement_summary(modeSummaryTbl)
    fprintf('\n');
    fprintf('================ FINAL REQUIRED SURFACE DEFLECTION SUMMARY ================\n');
    fprintf('%-28s %12s %14s %14s %18s %18s\n', ...
        'Flight mode', 'Target deg/s', 'Right Ail', 'Left Ail', 'Right Rudderv', 'Left Rudderv');
    fprintf('%s\n', repmat('-',1,110));

    for i = 1:height(modeSummaryTbl)
        fprintf('%-28s %12.3f %14.3f %14.3f %18.3f %18.3f\n', ...
            modeSummaryTbl.Mode{i}, ...
            modeSummaryTbl.TargetRate_degps(i), ...
            modeSummaryTbl.RightAileron_deg(i), ...
            modeSummaryTbl.LeftAileron_deg(i), ...
            modeSummaryTbl.RightRuddervator_deg(i), ...
            modeSummaryTbl.LeftRuddervator_deg(i));
    end

    fprintf('%s\n', repmat('-',1,110));
    fprintf('Signs above are the commanded PHYSICAL surface deflections [deg].\n');
    fprintf('This block is printed last so it stays at the bottom of the command window.\n');
end

function export_results_to_excel(resultsFile, inputsTbl, geometryTbl, modeSummaryTbl, derivTbl, ftTbl, stTbl, hmTbl)
    safeDelete(resultsFile);

    writetable(inputsTbl,      resultsFile, 'Sheet', 'Inputs');
    writetable(geometryTbl,    resultsFile, 'Sheet', 'Geometry');
    writetable(modeSummaryTbl, resultsFile, 'Sheet', 'ModeSummary');
    writetable(derivTbl,       resultsFile, 'Sheet', 'Derivatives');
    writetable(ftTbl,          resultsFile, 'Sheet', 'FT_Cases');
    writetable(stTbl,          resultsFile, 'Sheet', 'ST_Baseline');

    if isempty(hmTbl)
        hmTbl = empty_hm_table();
    end
    writetable(hmTbl,          resultsFile, 'Sheet', 'HM_All');
end

function safeDelete(fname)
    if isfile(fname)
        delete(fname);
    end
end

function touch_empty(fname)
    fid = fopen(fname,'w');
    if fid >= 0
        fclose(fid);
    else
        error("Could not create placeholder file: %s", fname);
    end
end

function write_airfoil(fid, airfoil_type, naca_code, afile)
    switch upper(string(airfoil_type))
        case "NACA"
            fprintf(fid,"NACA\n");
            fprintf(fid,"%s\n\n", string(naca_code));
        case "AFILE"
            fprintf(fid,"AFILE\n");
            fprintf(fid,"%s\n\n", string(afile));
        otherwise
            error("Unsupported airfoil type: %s", string(airfoil_type));
    end
end

function [x, y, z, c] = tail_station_from_eta(eta, XtailLE_root, LambdaLE_deg, b_panel, gamma_deg, c_root_vt, c_tip_vt)
    y = eta * b_panel * cosd(gamma_deg);
    z = eta * b_panel * sind(gamma_deg);
    c = c_root_vt + (c_tip_vt - c_root_vt)*eta;
    x = XtailLE_root + y*tand(LambdaLE_deg);
end

function write_vtail_surface(fid, surfaceName, ...
    y0, z0, c0, x0, ...
    y1, z1, c1, x1, ...
    y2, z2, c2, x2, ...
    y3, z3, c3, x3, ...
    airfoilType, nacaCode, Xhinge_rv, pitch_gain, yaw_gain)

    fprintf(fid,"SURFACE\n%s\n", surfaceName);
    fprintf(fid,"8 1.0 16 1.0\n\n");

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x0, y0, z0, c0);
    write_airfoil(fid, airfoilType, nacaCode, "");

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x1, y1, z1, c1);
    write_airfoil(fid, airfoilType, nacaCode, "");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"pitchv %.3f %.3f 0 0 0 1.0\n", pitch_gain, Xhinge_rv);
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"yawv %.3f %.3f 0 0 0 1.0\n\n", yaw_gain, Xhinge_rv);

    fprintf(fid,"SECTION\n");
    fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x2, y2, z2, c2);
    write_airfoil(fid, airfoilType, nacaCode, "");
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"pitchv %.3f %.3f 0 0 0 1.0\n", pitch_gain, Xhinge_rv);
    fprintf(fid,"CONTROL\n");
    fprintf(fid,"yawv %.3f %.3f 0 0 0 1.0\n\n", yaw_gain, Xhinge_rv);

    if abs(x3-x2) > 1e-9 || abs(y3-y2) > 1e-9 || abs(z3-z2) > 1e-9 || abs(c3-c2) > 1e-9
        fprintf(fid,"SECTION\n");
        fprintf(fid,"%.4f %.4f %.4f %.4f 0.0000\n", x3, y3, z3, c3);
        write_airfoil(fid, airfoilType, nacaCode, "");
    end
end