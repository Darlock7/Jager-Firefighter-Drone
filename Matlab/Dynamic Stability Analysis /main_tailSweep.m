%% ============================================================
% main_tailSweep.m
%
% Sweeps V-tail AR, taper, and LambdaLE_deg while keeping
% S_panel constant. Runs AVL for each case and keeps the
% best case based on handling-quality targets.
% ============================================================

clear; clc; close all;

%% =========================
% 1) CONSTANT INPUT STRUCT
% =========================
p = struct();

% ---------------- Files / AVL ----------------
p.avlExe = "avl352.exe";

% ---------------- Global constants ----------------
p.Mach = 0.00;
p.CDp  = 0.02813;

% ---------------- Aircraft weight / CG / inertia ----------------
p.W_lbf  = 55;
p.Xcg_in = 39.1;
p.Ycg_ft = 0.0;
p.Zcg_ft = -1.74/12;

p.Ixx_sw = 75.7671;
p.Iyy_sw = 119.6108;
p.Izz_sw = 191.5794;

% ---------------- Reference geometry ----------------
p.Sref = 6.64;
p.Cref = 1.052;
p.Bref = 6.31;

% ---------------- Wing geometry ----------------
p.XwingLE_in = 35.0;
p.eta_a0     = 0.60;
p.wing_naca  = "4412";

p.Xhinge_ail     = 0.75;
p.aileron_gain   = 1.0;
p.aileron_SgnDup = -1.0;

% ---------------- V-tail constants ----------------
p.S_panel    = 0.809;   % keep constant
p.gamma_deg  = 36.38;
p.XtailQC    = 6.696971667;
p.eta_rv0    = 0.30;
p.vtail_naca = "0010";

p.Xhinge_rv    = 0.70;
p.rudderv_gain = 1.0;

% ---------------- Flight condition ----------------
p.V_ftps    = 75.0;
p.alpha_deg = 8.25;
p.beta_deg  = 0.0;

%% =========================
% 2) HANDLING-QUALITY TARGETS
% =========================
% Treat Jager as:
%   - Class I aircraft
%   - Flight Phase B
%   - Level 1 objective

% -------- Longitudinal --------
p.target.phugoid_zeta_min      = 0.04;   % Level 1 minimum
p.target.shortPeriod_zeta_min  = 0.30;   % Phase B Level 1 minimum
p.target.shortPeriod_zeta_goal = 0.30;   % design target

% -------- Lateral --------
p.target.roll_tau_max          = 1.40;   % roll subsidence time constant max [s]
p.target.roll_tau_goal         = 1.00;   % preferred design-center value [s]

p.target.spiral_tDouble_min    = 20.0;   % minimum time to double [s]
p.target.spiral_tDouble_goal   = 25.0;   % preferred design-center value [s]

p.target.dutch_zeta_min        = 0.08;
p.target.dutch_zeta_goal       = 0.12;

p.target.dutch_zetaWn_min      = 0.15;
p.target.dutch_zetaWn_goal     = 0.20;

p.target.dutch_wn_min          = 0.40;   % rad/s
p.target.dutch_wn_goal         = 0.55;   % rad/s

% -------- Score weights --------
% Larger weights punish requirement violations.
% Smaller weights break ties among already-passing cases.
p.scoreW.phugoid_fail          = 45;
p.scoreW.short_fail            = 20;
p.scoreW.roll_fail             = 10;
p.scoreW.spiral_fail           = 45;
p.scoreW.dutch_fail_zeta       = 30;
p.scoreW.dutch_fail_zetaWn     = 25;
p.scoreW.dutch_fail_wn         = 15;

p.scoreW.short_goal            = 2;
p.scoreW.roll_goal             = 1;
p.scoreW.spiral_goal           = 4;
p.scoreW.dutch_goal_zeta       = 4;
p.scoreW.dutch_goal_zetaWn     = 2;
p.scoreW.dutch_goal_wn         = 1;

%% =========================
% 3) SWEEP RANGES
% =========================
AR_vec       = linspace(2.0, 2.8, 2); %was 3 3.5 might break CAD
taper_vec    = linspace(0.4, 0.6, 3);
LambdaLE_vec = linspace(30.0, 45.0, 3);

nAR  = numel(AR_vec);
nT   = numel(taper_vec);
nL   = numel(LambdaLE_vec);
Ntot = nAR * nT * nL;

fprintf('\nStarting V-tail geometry sweep...\n');
fprintf('AR points       : %d\n', nAR);
fprintf('Taper points    : %d\n', nT);
fprintf('LambdaLE points : %d\n', nL);
fprintf('Total cases     : %d\n\n', Ntot);

%% =========================
% 4) STORAGE
% =========================
results = struct([]);
kStore = 0;

best = struct();
best.found = false;
best.modeScore = inf;

%% =========================
% 5) SWEEP LOOP
% =========================
caseCounter = 0;
tStart = tic;

for i = 1:nAR
    for j = 1:nT
        for k = 1:nL

            caseCounter = caseCounter + 1;

            p.AR_tail      = AR_vec(i);
            p.taper_vt     = taper_vec(j);
            p.LambdaLE_deg = LambdaLE_vec(k);

            out = runAVL_tailSweepCase(p);

            pct = 100 * caseCounter / Ntot;

            if out.success
                % Re-score using explicit handling-quality targets
                [out.modeScore, out.modeBreakdown] = computeHandlingQualityScore(out, p);

                kStore = kStore + 1;

                results(kStore).AR_tail      = p.AR_tail;
                results(kStore).taper_vt     = p.taper_vt;
                results(kStore).LambdaLE_deg = p.LambdaLE_deg;
                results(kStore).modeScore    = out.modeScore;

                results(kStore).SP_score  = out.modeBreakdown.shortPeriod;
                results(kStore).PH_score  = out.modeBreakdown.phugoid;
                results(kStore).DR_score  = out.modeBreakdown.dutchRoll;
                results(kStore).RS_score  = out.modeBreakdown.rollSubsidence;
                results(kStore).SPR_score = out.modeBreakdown.spiral;

                results(kStore).lamLong = out.lamLong;
                results(kStore).lamLat  = out.lamLat;

                results(kStore).longModes = out.longModes;
                results(kStore).latModes  = out.latModes;

                if ~best.found || out.modeScore < best.modeScore
                    best = out;
                    best.found = true;

                    fprintf(['NEW BEST  | Case %4d / %4d | %6.2f%% | ' ...
                             'AR=%6.3f | taper=%6.3f | Lambda=%6.2f deg | ' ...
                             'Score=%12.4f\n'], ...
                             caseCounter, Ntot, pct, ...
                             p.AR_tail, p.taper_vt, p.LambdaLE_deg, ...
                             out.modeScore);
                else
                    fprintf(['Case %4d / %4d | %6.2f%% | ' ...
                             'AR=%6.3f | taper=%6.3f | Lambda=%6.2f deg | ' ...
                             'Score=%12.4f\n'], ...
                             caseCounter, Ntot, pct, ...
                             p.AR_tail, p.taper_vt, p.LambdaLE_deg, ...
                             out.modeScore);
                end
            else
                fprintf(['Case %4d / %4d | %6.2f%% | ' ...
                         'AR=%6.3f | taper=%6.3f | Lambda=%6.2f deg | FAILED\n'], ...
                         caseCounter, Ntot, pct, ...
                         p.AR_tail, p.taper_vt, p.LambdaLE_deg);
                fprintf('   Reason: %s\n', out.errMsg);
            end
        end
    end
end

elapsed = toc(tStart);

%% =========================
% 6) FINAL REPORT
% =========================
fprintf('\n============================================================\n');
fprintf('V-TAIL SWEEP COMPLETE\n');
fprintf('Elapsed time      : %.1f s\n', elapsed);
fprintf('Successful cases  : %d / %d\n', numel(results), Ntot);
fprintf('============================================================\n');

if ~best.found
    error('No successful AVL cases were found.');
end

fprintf('\n================ BEST TAIL GEOMETRY =================\n');
fprintf('S_panel      = %.4f ft^2 (held constant)\n', best.tail.S_panel);
fprintf('AR_tail      = %.4f\n', best.tail.AR_tail);
fprintf('taper_vt     = %.4f\n', best.tail.taper_vt);
fprintf('LambdaLE_deg = %.4f deg\n', best.tail.LambdaLE_deg);
fprintf('gamma_deg    = %.4f deg\n', best.tail.gamma_deg);
fprintf('b_panel      = %.4f ft\n', best.tail.b_panel);
fprintf('c_root_vt    = %.4f ft\n', best.tail.c_root_vt);
fprintf('c_tip_vt     = %.4f ft\n', best.tail.c_tip_vt);

fprintf('\n================ BEST MODE SCORE ====================\n');
fprintf('Total score        = %.6f\n', best.modeScore);
fprintf('Short Period score = %.6f\n', best.modeBreakdown.shortPeriod);
fprintf('Phugoid score      = %.6f\n', best.modeBreakdown.phugoid);
fprintf('Dutch Roll score   = %.6f\n', best.modeBreakdown.dutchRoll);
fprintf('Roll Mode score    = %.6f\n', best.modeBreakdown.rollSubsidence);
fprintf('Spiral score       = %.6f\n', best.modeBreakdown.spiral);

fprintf('\n================ BEST LONGITUDINAL MODES ============\n');
print_longitudinal_handling(best.longModes);

fprintf('\n================ BEST LATERAL MODES =================\n');
print_lateral_handling(best.latModes);

fprintf('\n================ BEST LONGITUDINAL EIGENVALUES ======\n');
disp(best.lamLong);

fprintf('\n================ BEST LATERAL EIGENVALUES ===========\n');
disp(best.lamLat);

%% =========================
% 7) BUILD SUMMARY TABLE
% =========================
if ~isempty(results)
    n = numel(results);

    AR_col    = zeros(n,1);
    T_col     = zeros(n,1);
    L_col     = zeros(n,1);
    score_col = zeros(n,1);
    SP_col    = zeros(n,1);
    PH_col    = zeros(n,1);
    DR_col    = zeros(n,1);
    RS_col    = zeros(n,1);
    SPR_col   = zeros(n,1);

    for ii = 1:n
        AR_col(ii)    = results(ii).AR_tail;
        T_col(ii)     = results(ii).taper_vt;
        L_col(ii)     = results(ii).LambdaLE_deg;
        score_col(ii) = results(ii).modeScore;
        SP_col(ii)    = results(ii).SP_score;
        PH_col(ii)    = results(ii).PH_score;
        DR_col(ii)    = results(ii).DR_score;
        RS_col(ii)    = results(ii).RS_score;
        SPR_col(ii)   = results(ii).SPR_score;
    end

    T = table(AR_col, T_col, L_col, score_col, SP_col, PH_col, DR_col, RS_col, SPR_col, ...
        'VariableNames', {'AR_tail','taper_vt','LambdaLE_deg','modeScore', ...
                          'SP_score','PH_score','DR_score','RS_score','SPR_score'});

    T = sortrows(T, 'modeScore', 'ascend');

    fprintf('\n================ TOP 15 CASES =======================\n');
    disp(T(1:min(15,height(T)), :));

    assignin('base', 'tailSweepResultsTable', T);
    assignin('base', 'tailSweepBestCase', best);
end

%% =========================
% 8) OPTIONAL SIMPLE VISUALS
% =========================
if exist('T', 'var') && height(T) > 0
    figure('Color','w');
    scatter3(T.AR_tail, T.taper_vt, T.LambdaLE_deg, 50, T.modeScore, 'filled');
    grid on;
    xlabel('AR tail');
    ylabel('Taper');
    zlabel('\Lambda_{LE} (deg)');
    title('V-tail sweep colored by mode score');
    colorbar;
end

%% =========================
% 9) LOCAL PRINT FUNCTIONS
% =========================
function print_longitudinal_handling(longModes)
    sp = longModes.shortPeriod.metrics;
    ph = longModes.phugoid.metrics;

    fprintf("Short Period:\n");
    fprintf("  lambda      = %+.4f %+.4fi\n", real(sp.lambda), imag(sp.lambda));
    fprintf("  wn          = %.4f rad/s\n", sp.wn);
    fprintf("  zeta        = %.4f\n", sp.zeta);
    fprintf("  period      = %.4f s\n", sp.T);
    fprintf("  tau         = %.4f s\n", sp.tau);
    if ~isnan(sp.ts2)
        fprintf("  ts(2%%)      = %.4f s\n", sp.ts2);
    end

    fprintf("\nPhugoid:\n");
    fprintf("  lambda      = %+.4f %+.4fi\n", real(ph.lambda), imag(ph.lambda));
    fprintf("  wn          = %.4f rad/s\n", ph.wn);
    fprintf("  zeta        = %.4f\n", ph.zeta);
    fprintf("  period      = %.4f s\n", ph.T);
    fprintf("  tau         = %.4f s\n", ph.tau);
    if ~isnan(ph.ts2)
        fprintf("  ts(2%%)      = %.4f s\n", ph.ts2);
    end
end

function print_lateral_handling(latModes)
    dr  = latModes.dutchRoll.metrics;
    rs  = latModes.rollSubsidence.metrics;
    spr = latModes.spiral.metrics;

    fprintf("Dutch Roll:\n");
    fprintf("  lambda      = %+.4f %+.4fi\n", real(dr.lambda), imag(dr.lambda));
    fprintf("  wn          = %.4f rad/s\n", dr.wn);
    fprintf("  zeta        = %.4f\n", dr.zeta);
    fprintf("  zeta*wn     = %.4f\n", dr.zeta * dr.wn);
    fprintf("  period      = %.4f s\n", dr.T);
    fprintf("  tau         = %.4f s\n", dr.tau);
    if ~isnan(dr.ts2)
        fprintf("  ts(2%%)      = %.4f s\n", dr.ts2);
    end

    fprintf("\nRoll Subsidence:\n");
    fprintf("  lambda      = %+.4f\n", real(rs.lambda));
    fprintf("  tau         = %.4f s\n", rs.tau);
    if ~isnan(rs.tHalf)
        fprintf("  t_half      = %.4f s\n", rs.tHalf);
    end
    if ~isnan(rs.tDouble)
        fprintf("  t_double    = %.4f s\n", rs.tDouble);
    end

    fprintf("\nSpiral Mode:\n");
    fprintf("  lambda      = %+.4f\n", real(spr.lambda));
    fprintf("  tau         = %.4f s\n", spr.tau);
    if ~isnan(spr.tHalf)
        fprintf("  t_half      = %.4f s\n", spr.tHalf);
    end
    if ~isnan(spr.tDouble)
        fprintf("  t_double    = %.4f s\n", spr.tDouble);
    end
end

%% =========================
% 10) LOCAL SCORE FUNCTIONS
% =========================
function [score, breakdown] = computeHandlingQualityScore(out, p)

    sp  = out.longModes.shortPeriod.metrics;
    ph  = out.longModes.phugoid.metrics;
    dr  = out.latModes.dutchRoll.metrics;
    rs  = out.latModes.rollSubsidence.metrics;
    spr = out.latModes.spiral.metrics;

    breakdown = struct();
    breakdown.shortPeriod    = 0;
    breakdown.phugoid        = 0;
    breakdown.dutchRoll      = 0;
    breakdown.rollSubsidence = 0;
    breakdown.spiral         = 0;

    % -------------------------
    % 1) PHUGOID
    % Require zeta >= 0.04
    % -------------------------
    breakdown.phugoid = breakdown.phugoid + ...
        p.scoreW.phugoid_fail * deficitPenalty(ph.zeta, p.target.phugoid_zeta_min);

    % -------------------------
    % 2) SHORT PERIOD
    % Require zeta >= 0.30
    % Prefer near target 0.30
    % -------------------------
    breakdown.shortPeriod = breakdown.shortPeriod + ...
        p.scoreW.short_fail * deficitPenalty(sp.zeta, p.target.shortPeriod_zeta_min);

    breakdown.shortPeriod = breakdown.shortPeriod + ...
        p.scoreW.short_goal * closenessPenaltyAboveMin( ...
        sp.zeta, p.target.shortPeriod_zeta_min, p.target.shortPeriod_zeta_goal);

    % -------------------------
    % 3) ROLL SUBSIDENCE
    % Require tau <= 1.4 s
    % -------------------------
    breakdown.rollSubsidence = breakdown.rollSubsidence + ...
        p.scoreW.roll_fail * excessPenalty(rs.tau, p.target.roll_tau_max);

    breakdown.rollSubsidence = breakdown.rollSubsidence + ...
        p.scoreW.roll_goal * closenessPenaltyBelowMax( ...
        rs.tau, p.target.roll_tau_max, p.target.roll_tau_goal);

    % -------------------------
    % 4) SPIRAL
    % Require time-to-double >= 20 s
    % Stable spiral (negative real lambda) gets zero fail penalty
    % -------------------------
    if ~isnan(spr.tDouble)
        breakdown.spiral = breakdown.spiral + ...
            p.scoreW.spiral_fail * deficitPenalty(spr.tDouble, p.target.spiral_tDouble_min);

        breakdown.spiral = breakdown.spiral + ...
            p.scoreW.spiral_goal * closenessPenaltyAboveMin( ...
            spr.tDouble, p.target.spiral_tDouble_min, p.target.spiral_tDouble_goal);
    else
        if real(spr.lambda) < 0
            breakdown.spiral = breakdown.spiral + 0;
        else
            breakdown.spiral = breakdown.spiral + 1e3;
        end
    end

    % -------------------------
    % 5) DUTCH ROLL
    % Require:
    %   zeta      >= 0.08
    %   zeta*wn   >= 0.15
    %   wn        >= 0.4 rad/s
    % -------------------------
    dr_zetaWn = dr.zeta * dr.wn;

    breakdown.dutchRoll = breakdown.dutchRoll + ...
        p.scoreW.dutch_fail_zeta * deficitPenalty(dr.zeta, p.target.dutch_zeta_min);

    breakdown.dutchRoll = breakdown.dutchRoll + ...
        p.scoreW.dutch_fail_zetaWn * deficitPenalty(dr_zetaWn, p.target.dutch_zetaWn_min);

    breakdown.dutchRoll = breakdown.dutchRoll + ...
        p.scoreW.dutch_fail_wn * deficitPenalty(dr.wn, p.target.dutch_wn_min);

    breakdown.dutchRoll = breakdown.dutchRoll + ...
        p.scoreW.dutch_goal_zeta * closenessPenaltyAboveMin( ...
        dr.zeta, p.target.dutch_zeta_min, p.target.dutch_zeta_goal);

    breakdown.dutchRoll = breakdown.dutchRoll + ...
        p.scoreW.dutch_goal_zetaWn * closenessPenaltyAboveMin( ...
        dr_zetaWn, p.target.dutch_zetaWn_min, p.target.dutch_zetaWn_goal);

    breakdown.dutchRoll = breakdown.dutchRoll + ...
        p.scoreW.dutch_goal_wn * closenessPenaltyAboveMin( ...
        dr.wn, p.target.dutch_wn_min, p.target.dutch_wn_goal);

    score = breakdown.shortPeriod + breakdown.phugoid + ...
            breakdown.dutchRoll + breakdown.rollSubsidence + ...
            breakdown.spiral;
end

function pval = deficitPenalty(val, minVal)
    % 0 if val >= minVal
    % positive if val < minVal
    if isnan(val) || isinf(val)
        pval = 1e3;
    else
        pval = max(0, (minVal - val) / max(minVal, 1e-9))^2;
    end
end

function pval = excessPenalty(val, maxVal)
    % 0 if val <= maxVal
    % positive if val > maxVal
    if isnan(val) || isinf(val)
        pval = 1e3;
    else
        pval = max(0, (val - maxVal) / max(maxVal, 1e-9))^2;
    end
end

function pval = closenessPenaltyAboveMin(val, minVal, goalVal)
    % No tie-break penalty until requirement is passed.
    % Among passing cases, prefer being near goalVal.
    if isnan(val) || isinf(val) || val < minVal
        pval = 0;
    else
        denom = max(abs(goalVal), 1e-9);
        pval = ((val - goalVal) / denom)^2;
    end
end

function pval = closenessPenaltyBelowMax(val, maxVal, goalVal)
    % No tie-break penalty until requirement is passed.
    % Among passing cases, prefer being near goalVal.
    if isnan(val) || isinf(val) || val > maxVal
        pval = 0;
    else
        denom = max(abs(goalVal), 1e-9);
        pval = ((val - goalVal) / denom)^2;
    end
end