function out = runAVL_tailSweepCase(p)
% ============================================================
% runAVL_tailSweepCase.m
%
% Runs one AVL case for a given V-tail geometry and returns:
%   - longitudinal/lateral eigenvalues
%   - canonical mode classification
%   - mode metrics
%   - mode-based optimization score
%
% INPUT:
%   p : struct with constants and swept tail vars
%
% REQUIRED SWEPT INPUTS:
%   p.AR_tail
%   p.taper_vt
%   p.LambdaLE_deg
%
% REQUIRED FIXED INPUTS:
%   p.S_panel
%   p.gamma_deg
%   p.XtailQC
%   p.eta_rv0
%   p.avlExe
%   ... plus aircraft / trim / geometry constants
%
% OUTPUT:
%   out.success
%   out.errMsg
%   out.lamLong
%   out.lamLat
%   out.longModes
%   out.latModes
%   out.modeScore
%   out.modeBreakdown
%   out.Along
%   out.Alat
%   out.tail
% ============================================================

out = struct();
out.success = false;
out.errMsg  = "";

try
    %% =========================
    % 0) UNIQUE TEMP FILENAMES
    % =========================
    caseTag = sprintf('AR_%0.4f_T_%0.4f_L_%0.4f', ...
        p.AR_tail, p.taper_vt, p.LambdaLE_deg);
    caseTag = strrep(caseTag, '.', 'p');

    geomFile = ['tmp_' caseTag '_jager.geo.avl'];
    massFile = ['tmp_' caseTag '_jager.mass'];
    caseFile = ['tmp_' caseTag '_jager.run'];
    stFile   = ['tmp_' caseTag '_jager_st.txt'];
    cmdFile  = ['tmp_' caseTag '_jager_cmd.txt'];

    %% =========================
    % 1) GLOBAL CONSTANTS / UNITS
    % =========================
    g_ft = 32.174;

    avlExe = p.avlExe;
    Mach   = p.Mach;
    CDp    = p.CDp;

    %% =========================
    % 2) AIRCRAFT WEIGHT / CG / INERTIA
    % =========================
    W_lbf  = p.W_lbf;
    m_slug = W_lbf / g_ft;

    Xcg_in = p.Xcg_in;
    Ycg_ft = p.Ycg_ft;
    Zcg_ft = p.Zcg_ft;
    Xcg_ft = Xcg_in / 12;

    convI = 1 / 32.174;
    Ixx = p.Ixx_sw * convI;
    Iyy = p.Iyy_sw * convI;
    Izz = p.Izz_sw * convI;

    % principal-axes assumption
    Ixy = 0;
    Ixz = 0;
    Iyz = 0;

    %% =========================
    % 3) REFERENCE GEOMETRY
    % =========================
    Sref = p.Sref;
    Cref = p.Cref;
    Bref = p.Bref;
    b2   = Bref / 2;

    Xref = Xcg_ft;
    Yref = 0.0;
    Zref = Zcg_ft;

    %% =========================
    % 4) WING GEOMETRY
    % =========================
    XwingLE = p.XwingLE_in / 12;
    c_w     = Cref;

    eta_a0 = p.eta_a0;
    y_a0   = eta_a0 * b2;
    y_tip  = 1.00 * b2;

    wing_airfoil_type = "NACA";
    wing_naca         = p.wing_naca;

    Xhinge_ail     = p.Xhinge_ail;
    aileron_gain   = p.aileron_gain;
    aileron_SgnDup = p.aileron_SgnDup;

    %% =========================
    % 5) V-TAIL GEOMETRY
    % =========================
    S_panel      = p.S_panel;        % constant during sweep
    AR_tail      = p.AR_tail;        % swept
    taper_vt     = p.taper_vt;       % swept
    gamma_deg    = p.gamma_deg;      % fixed
    LambdaLE_deg = p.LambdaLE_deg;   % swept
    XtailQC      = p.XtailQC;        % fixed

    b_panel   = sqrt(AR_tail * S_panel);
    c_root_vt = 2 * S_panel / (b_panel * (1 + taper_vt));
    c_tip_vt  = taper_vt * c_root_vt;

    XtailLE_root = XtailQC - 0.25 * c_root_vt;

    yT = b_panel * cosd(gamma_deg);
    zT = b_panel * sind(gamma_deg);

    eta_rv0 = p.eta_rv0;

    vtail_airfoil_type = "NACA";
    vtail_naca         = p.vtail_naca;

    Xhinge_rv    = p.Xhinge_rv;
    rudderv_gain = p.rudderv_gain;

    %% =========================
    % 6) FLIGHT CONDITION
    % =========================
    V_ftps    = p.V_ftps;
    alpha_deg = p.alpha_deg;
    beta_deg  = p.beta_deg;

    %% =========================
    % 7) WRITE GEOMETRY FILE
    % =========================
    fid = fopen(geomFile, 'w');
    assert(fid > 0, "Could not write geometry file.");

    fprintf(fid, "JAGER Tail Sweep Case\n");
    fprintf(fid, "%.4f\n", Mach);
    fprintf(fid, "0 0 0.0\n");
    fprintf(fid, "%.4f %.4f %.4f\n", Sref, Cref, Bref);
    fprintf(fid, "%.4f %.4f %.4f\n", Xref, Yref, Zref);
    fprintf(fid, "%.6f\n\n", CDp);

    % ---------------- Wing ----------------
    fprintf(fid, "SURFACE\nWing\n");
    fprintf(fid, "10 1.0 24 1.0\n");
    fprintf(fid, "YDUPLICATE\n0.0\n\n");

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f 0.0000 0.0000 %.4f 0.0000\n", XwingLE, c_w);
    write_airfoil(fid, wing_airfoil_type, wing_naca, "");

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_a0, c_w);
    write_airfoil(fid, wing_airfoil_type, wing_naca, "");
    fprintf(fid, "CONTROL\n");
    fprintf(fid, "aileron %.3f %.3f 0 0 0 %.1f\n\n", ...
        aileron_gain, Xhinge_ail, aileron_SgnDup);

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f 0.0000 %.4f 0.0000\n", XwingLE, y_tip, c_w);
    write_airfoil(fid, wing_airfoil_type, wing_naca, "");
    fprintf(fid, "CONTROL\n");
    fprintf(fid, "aileron %.3f %.3f 0 0 0 %.1f\n\n", ...
        aileron_gain, Xhinge_ail, aileron_SgnDup);

    % ---------------- V-tail shared stations ----------------
    eta0 = 0.0;
    eta1 = eta_rv0;
    eta2 = 1.0;

    y0p = eta0 * abs(yT);  z0 = eta0 * zT;
    y1p = eta1 * abs(yT);  z1 = eta1 * zT;
    y2p = eta2 * abs(yT);  z2 = eta2 * zT;

    c0 = c_root_vt + (c_tip_vt - c_root_vt) * eta0;
    c1 = c_root_vt + (c_tip_vt - c_root_vt) * eta1;
    c2 = c_root_vt + (c_tip_vt - c_root_vt) * eta2;

    x0 = XtailLE_root + y0p * tand(LambdaLE_deg);
    x1 = XtailLE_root + y1p * tand(LambdaLE_deg);
    x2 = XtailLE_root + y2p * tand(LambdaLE_deg);

    % ---------------- Vtail Right ----------------
    fprintf(fid, "SURFACE\nVtail_R\n");
    fprintf(fid, "8 1.0 16 1.0\n\n");

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f %.4f %.4f 0.0000\n", x0, +y0p, z0, c0);
    write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f %.4f %.4f 0.0000\n", x1, +y1p, z1, c1);
    write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");
    fprintf(fid, "CONTROL\n");
    fprintf(fid, "rudderv %.3f %.3f 0 0 0 1.0\n\n", ...
        rudderv_gain, Xhinge_rv);

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f %.4f %.4f 0.0000\n", x2, +y2p, z2, c2);
    write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");
    fprintf(fid, "CONTROL\n");
    fprintf(fid, "rudderv %.3f %.3f 0 0 0 1.0\n\n", ...
        rudderv_gain, Xhinge_rv);

    % ---------------- Vtail Left ----------------
    fprintf(fid, "SURFACE\nVtail_L\n");
    fprintf(fid, "8 1.0 16 1.0\n\n");

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f %.4f %.4f 0.0000\n", x0, -y0p, z0, c0);
    write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f %.4f %.4f 0.0000\n", x1, -y1p, z1, c1);
    write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");
    fprintf(fid, "CONTROL\n");
    fprintf(fid, "rudderv %.3f %.3f 0 0 0 1.0\n\n", ...
        rudderv_gain, Xhinge_rv);

    fprintf(fid, "SECTION\n");
    fprintf(fid, "%.4f %.4f %.4f %.4f 0.0000\n", x2, -y2p, z2, c2);
    write_airfoil(fid, vtail_airfoil_type, vtail_naca, "");
    fprintf(fid, "CONTROL\n");
    fprintf(fid, "rudderv %.3f %.3f 0 0 0 1.0\n\n", ...
        rudderv_gain, Xhinge_rv);

    fclose(fid);

    %% =========================
    % 8) WRITE MASS FILE
    % =========================
    fid = fopen(massFile, 'w');
    assert(fid > 0, "Could not write mass file.");

    fprintf(fid, "# mass  x  y  z  Ixx  Iyy  Izz  Ixz  Ixy  Iyz\n");
    fprintf(fid, "# mass units: slug\n");
    fprintf(fid, "# length units: ft\n");
    fprintf(fid, "# inertia units: slug*ft^2\n");
    fprintf(fid, "%.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f  %.6f\n", ...
        m_slug, Xcg_ft, Ycg_ft, Zcg_ft, Ixx, Iyy, Izz, Ixz, Ixy, Iyz);
    fclose(fid);

    %% =========================
    % 9) WRITE RUN CASE FILE
    % =========================
    fid = fopen(caseFile, 'w');
    assert(fid > 0, "Could not write run-case file.");
    fprintf(fid, "# alpha[deg]  beta[deg]  V[ft/s]\n");
    fprintf(fid, "%.6f  %.6f  %.6f\n", alpha_deg, beta_deg, V_ftps);
    fclose(fid);

    %% =========================
    % 10) WRITE AVL COMMAND FILE
    % =========================
    if isfile(stFile)
        delete(stFile);
    end

    fid = fopen(cmdFile, 'w');
    assert(fid > 0, "Could not write AVL command file.");

    fprintf(fid, "LOAD %s\n", geomFile);
    fprintf(fid, "MASS %s\n", massFile);
    fprintf(fid, "CASE %s\n", caseFile);
    fprintf(fid, "MSET\n");
    fprintf(fid, "0\n");
    fprintf(fid, "OPER\n");
    fprintf(fid, "A %.6f\n", alpha_deg);
    fprintf(fid, "A %.6f\n", alpha_deg);
    fprintf(fid, "X\n");
    fprintf(fid, "ST\n");
    fprintf(fid, "%s\n", stFile);
    fprintf(fid, "\n");
    fprintf(fid, "QUIT\n");

    fclose(fid);

    %% =========================
    % 11) RUN AVL
    % =========================
    assert(isfile(avlExe), "AVL executable not found: %s", avlExe);

    syscmd = sprintf('"%s" < "%s"', avlExe, cmdFile);
    [status, cmdout] = system(syscmd);

    if ~isfile(stFile)
        error("ST output file was not created. AVL output:\n%s", cmdout);
    end

    if status ~= 0
        warning("AVL returned nonzero exit status (%d), continuing because ST file exists.", status);
    end

    %% =========================
    % 12) DYNAMICS SETUP
    % =========================
    g_ftps2 = 32.174;
    rho     = 0.0023769;

    u0_ftps    = V_ftps;
    theta0_deg = 0.0;
    theta0_rad = deg2rad(theta0_deg);

    Sref_ft2 = Sref;
    Cref_ft  = Cref;
    Bref_ft  = Bref;

    m_slug_dyn  = m_slug;
    Ix_slugft2  = Ixx;
    Iy_slugft2  = Iyy;
    Iz_slugft2  = Izz;
    Ixz_slugft2 = Ixz;

    Zw_dot = 0.0;
    Mw_dot = 0.0;

    %% =========================
    % 13) PARSE ST FILE
    % =========================
    txt = fileread(stFile);
    get = @(pat) must_getnum(txt, pat);

    CL0 = get("CLtot =");
    CD0 = get("CDtot =");

    CLa = get("CLa =");
    CDa = get("CDa =");
    Cma = get("Cma =");

    CLq = get("CLq =");
    CDq = get("CDq =");
    Cmq = get("Cmq =");

    CYb = get("CYb =");
    Clb = get("Clb =");
    Cnb = get("Cnb =");

    CYp = get("CYp =");
    Clp = get("Clp =");
    Cnp = get("Cnp =");

    CYr = get("CYr =");
    Clr = get("Clr =");
    Cnr = get("Cnr =");

    %% =========================
    % 14) DIMENSIONALIZE DERIVATIVES
    % =========================
    qbar = 0.5 * rho * u0_ftps^2;

    Xu = -rho * u0_ftps * Sref_ft2 * CD0;
    Zu = -rho * u0_ftps * Sref_ft2 * CL0;

    dX_dalpha = -qbar * Sref_ft2 * CDa;
    dZ_dalpha = -qbar * Sref_ft2 * CLa;

    Xw = (1 / u0_ftps) * dX_dalpha;
    Zw = (1 / u0_ftps) * dZ_dalpha;

    dCL_dq = CLq * (Cref_ft / (2 * u0_ftps));
    dCD_dq = CDq * (Cref_ft / (2 * u0_ftps));

    Zq = -qbar * Sref_ft2 * dCL_dq;
    Xq = -qbar * Sref_ft2 * dCD_dq;

    dM_dalpha = qbar * Sref_ft2 * Cref_ft * Cma;
    Mw = (1 / u0_ftps) * dM_dalpha;

    dCm_dq = Cmq * (Cref_ft / (2 * u0_ftps));
    Mq = qbar * Sref_ft2 * Cref_ft * dCm_dq;

    Mu = 0.0;

    Yv = (qbar * Sref_ft2)         * (CYb / u0_ftps);
    Lv = (qbar * Sref_ft2 * Bref_ft) * (Clb / u0_ftps);
    Nv = (qbar * Sref_ft2 * Bref_ft) * (Cnb / u0_ftps);

    Yp = (qbar * Sref_ft2)         * (CYp * (Bref_ft / (2 * u0_ftps)));
    Lp = (qbar * Sref_ft2 * Bref_ft) * (Clp * (Bref_ft / (2 * u0_ftps)));
    Np = (qbar * Sref_ft2 * Bref_ft) * (Cnp * (Bref_ft / (2 * u0_ftps)));

    Yr = (qbar * Sref_ft2)         * (CYr * (Bref_ft / (2 * u0_ftps)));
    Lr = (qbar * Sref_ft2 * Bref_ft) * (Clr * (Bref_ft / (2 * u0_ftps)));
    Nr = (qbar * Sref_ft2 * Bref_ft) * (Cnr * (Bref_ft / (2 * u0_ftps)));

    %% =========================
    % 15) BUILD STATE MATRICES
    % =========================
    den = (m_slug_dyn - Zw_dot);

    Along = zeros(4,4);
    Along(1,1) = Xu / m_slug_dyn;
    Along(1,2) = Xw / m_slug_dyn;
    Along(1,3) = Xq / m_slug_dyn;
    Along(1,4) = -g_ftps2 * cos(theta0_rad);

    Along(2,1) = Zu / den;
    Along(2,2) = Zw / den;
    Along(2,3) = (Zq + m_slug_dyn * u0_ftps) / den;
    Along(2,4) = -(m_slug_dyn * g_ftps2 * sin(theta0_rad)) / den;

    Along(3,1) = (1 / Iy_slugft2) * (Mu + (Mw_dot * Zu) / den);
    Along(3,2) = (1 / Iy_slugft2) * (Mw + (Mw_dot * Zw) / den);
    Along(3,3) = (1 / Iy_slugft2) * (Mq + (Mw_dot * (Zq + m_slug_dyn * u0_ftps)) / den);
    Along(4,3) = 1;

    D = (Ix_slugft2 * Iz_slugft2 - Ixz_slugft2^2);
    Ix_p  = D / Iz_slugft2;
    Iz_p  = D / Ix_slugft2;
    Izx_p = Ixz_slugft2 / D;

    Alat = zeros(4,4);
    Alat(1,1) = Yv / m_slug_dyn;
    Alat(1,2) = Yp / m_slug_dyn;
    Alat(1,3) = (Yr / m_slug_dyn) - u0_ftps;
    Alat(1,4) = g_ftps2 * cos(theta0_rad);

    Alat(2,1) = (Lv / Ix_p + Izx_p * Nv);
    Alat(2,2) = (Lp / Ix_p + Izx_p * Np);
    Alat(2,3) = (Lr / Ix_p + Izx_p * Nr);

    Alat(3,1) = (Izx_p * Lv + Nv / Iz_p);
    Alat(3,2) = (Izx_p * Lp + Np / Iz_p);
    Alat(3,3) = (Izx_p * Lr + Nr / Iz_p);

    Alat(4,2) = 1;
    Alat(4,3) = tan(theta0_rad);

    %% =========================
    % 16) EIGENANALYSIS
    % =========================
    [VL, DL] = eig(Along);
    lamLong  = diag(DL);

    [VT, DT] = eig(Alat);
    lamLat   = diag(DT);

    %% =========================
    % 17) MODE CLASSIFICATION
    % =========================
    longModes = classify_longitudinal_modes(lamLong, VL);
    latModes  = classify_lateral_modes(lamLat, VT);

    %% =========================
    % 18) MODE-BASED SCORE
    % =========================
    [modeScore, modeBreakdown] = total_mode_score(longModes, latModes, p);

    %% =========================
    % 19) PACK OUTPUT
    % =========================
    out.success = true;
    out.errMsg  = "";

    out.lamLong = lamLong;
    out.lamLat  = lamLat;
    out.Along   = Along;
    out.Alat    = Alat;

    out.longModes = longModes;
    out.latModes  = latModes;

    out.modeScore     = modeScore;
    out.modeBreakdown = modeBreakdown;

    out.tail.S_panel      = S_panel;
    out.tail.AR_tail      = AR_tail;
    out.tail.taper_vt     = taper_vt;
    out.tail.gamma_deg    = gamma_deg;
    out.tail.LambdaLE_deg = LambdaLE_deg;
    out.tail.XtailQC      = XtailQC;
    out.tail.b_panel      = b_panel;
    out.tail.c_root_vt    = c_root_vt;
    out.tail.c_tip_vt     = c_tip_vt;
    out.tail.yT           = yT;
    out.tail.zT           = zT;

catch ME
    out.success = false;
    out.errMsg  = string(ME.message);
end

%% =========================
% 20) CLEANUP
% =========================
delete_if_exists(geomFile);
delete_if_exists(massFile);
delete_if_exists(caseFile);
delete_if_exists(stFile);
delete_if_exists(cmdFile);

end

% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function write_airfoil(fid, typeStr, nacaStr, aFileStr)
    typeStr = upper(string(typeStr));
    if typeStr == "NACA"
        fprintf(fid, "NACA\n%s\n\n", string(nacaStr));
    elseif typeStr == "AFILE"
        if strlength(aFileStr) == 0
            error("AFILE selected but no airfoil filename provided.");
        end
        fprintf(fid, "AFILE\n%s\n\n", string(aFileStr));
    else
        error("Unknown airfoil type. Use 'NACA' or 'AFILE'.");
    end
end

function val = must_getnum(txt, pattern)
    expr = pattern + "\s*([-+]?(\d+(\.\d*)?|\.\d+)([eEdD][-+]?\d+)?)";
    tok = regexp(txt, expr, 'tokens', 'once');
    assert(~isempty(tok), "Could not find pattern: %s", pattern);
    val = str2double(strrep(tok{1}, 'D', 'E'));
end

function delete_if_exists(fname)
    if exist(fname, 'file') == 2
        delete(fname);
    end
end

function longModes = classify_longitudinal_modes(lam, eigvecs)
    idxC = find(abs(imag(lam)) > 1e-8);
    assert(numel(idxC) >= 2, 'Expected two complex longitudinal modes.');

    lamC = lam(idxC);
    vecC = eigvecs(:, idxC);

    [~, order] = sort(abs(imag(lamC)), 'descend');

    idxSP = order(1);
    idxPH = order(end);

    longModes.shortPeriod.lambda  = lamC(idxSP);
    longModes.shortPeriod.vec     = vecC(:, idxSP);
    longModes.shortPeriod.metrics = mode_metrics(lamC(idxSP));

    longModes.phugoid.lambda  = lamC(idxPH);
    longModes.phugoid.vec     = vecC(:, idxPH);
    longModes.phugoid.metrics = mode_metrics(lamC(idxPH));
end

function latModes = classify_lateral_modes(lam, eigvecs)
    idxC = find(abs(imag(lam)) > 1e-8);
    idxR = find(abs(imag(lam)) <= 1e-8);

    assert(~isempty(idxC), 'Expected Dutch roll complex pair.');
    assert(numel(idxR) >= 2, 'Expected two real lateral roots.');

    lamC = lam(idxC);
    vecC = eigvecs(:, idxC);

    [~, idxDR] = max(abs(imag(lamC)));
    latModes.dutchRoll.lambda  = lamC(idxDR);
    latModes.dutchRoll.vec     = vecC(:, idxDR);
    latModes.dutchRoll.metrics = mode_metrics(lamC(idxDR));

    lamR = real(lam(idxR));
    vecR = eigvecs(:, idxR);

    [~, idxMostNeg] = min(lamR);
    latModes.rollSubsidence.lambda  = lamR(idxMostNeg);
    latModes.rollSubsidence.vec     = vecR(:, idxMostNeg);
    latModes.rollSubsidence.metrics = mode_metrics(lamR(idxMostNeg));

    tmp = lamR;
    tmp(idxMostNeg) = inf;
    [~, idxClosestZero] = min(abs(tmp));

    spiralVal = tmp(idxClosestZero);
    spiralIdxGlobal = idxR(idxClosestZero);

    latModes.spiral.lambda  = spiralVal;
    latModes.spiral.vec     = eigvecs(:, spiralIdxGlobal);
    latModes.spiral.metrics = mode_metrics(spiralVal);
end

function m = mode_metrics(lam)
    m.lambda = lam;
    m.sigma  = real(lam);
    m.omega  = imag(lam);

    if abs(imag(lam)) > 1e-8
        m.isComplex = true;
        m.wn   = hypot(real(lam), imag(lam));
        m.zeta = -real(lam) / m.wn;
        m.T    = 2*pi / abs(imag(lam));
        m.tau  = 1 / abs(real(lam));

        if real(lam) < 0
            m.ts2     = 4 / abs(real(lam));
            m.tDouble = NaN;
            m.tHalf   = NaN;
        else
            m.ts2     = NaN;
            m.tDouble = log(2) / real(lam);
            m.tHalf   = NaN;
        end
    else
        m.isComplex = false;
        m.wn   = NaN;
        m.zeta = NaN;
        m.T    = NaN;

        if real(lam) ~= 0
            m.tau = 1 / abs(real(lam));
        else
            m.tau = Inf;
        end

        if real(lam) < 0
            m.tHalf   = log(2) / abs(real(lam));
            m.tDouble = NaN;
        elseif real(lam) > 0
            m.tDouble = log(2) / real(lam);
            m.tHalf   = NaN;
        else
            m.tDouble = Inf;
            m.tHalf   = Inf;
        end

        m.ts2 = NaN;
    end
end

function [score, breakdown] = total_mode_score(longModes, latModes, p)
    % Weighting:
    % Emphasize lateral-directional modes more heavily for V-tail sweep

    sp  = score_short_period(longModes.shortPeriod.metrics, p);
    ph  = score_phugoid(longModes.phugoid.metrics, p);
    dr  = score_dutch_roll(latModes.dutchRoll.metrics, p);
    rs  = score_roll_mode(latModes.rollSubsidence.metrics, p);
    spr = score_spiral(latModes.spiral.metrics, p);

    breakdown.shortPeriod    = sp;
    breakdown.phugoid        = ph;
    breakdown.dutchRoll      = dr;
    breakdown.rollSubsidence = rs;
    breakdown.spiral         = spr;

    score = sp + ph + dr + rs + spr;
end

function score = score_short_period(m, p)
    % Want:
    %   stable
    %   decent damping
    %   reasonable natural frequency
    %
    % Default target window based on your previous discussion:
    %   zeta >= 0.35 preferred
    %   wn not too low

    if isfield(p, 'w_shortPeriod')
        W = p.w_shortPeriod;
    else
        W = 4;
    end

    zeta_min = 0.35;
    zeta_max = 1.30;
    wn_min   = 1.0;
    wn_tgt   = 2.0;

    score = 0;

    if real(m.lambda) >= 0
        score = score + W * 1e6;
        return
    end

    if m.zeta < zeta_min
        score = score + W * 1000 * (zeta_min - m.zeta)^2;
    elseif m.zeta > zeta_max
        score = score + W * 100 * (m.zeta - zeta_max)^2;
    end

    if m.wn < wn_min
        score = score + W * 300 * (wn_min - m.wn)^2;
    else
        score = score + W * 5 * (m.wn - wn_tgt)^2;
    end
end

function score = score_phugoid(m, p)
    if isfield(p, 'w_phugoid')
        W = p.w_phugoid;
    else
        W = 2;
    end

    zeta_min = 0.04;

    score = 0;

    if real(m.lambda) >= 0
        score = score + W * 1e6;
        return
    end

    if m.zeta < zeta_min
        score = score + W * 500 * (zeta_min - m.zeta)^2;
    end
end

function score = score_dutch_roll(m, p)
    % Sadraey-style thresholds commonly used:
    %   zeta >= 0.08
    %   zeta*wn >= 0.15
    %   wn >= 0.4
    %
    % Strongly prioritize this mode for V-tail sweep.

    if isfield(p, 'w_dutchRoll')
        W = p.w_dutchRoll;
    else
        W = 10;
    end

    zeta_min   = 0.08;
    zetaWn_min = 0.15;
    wn_min     = 0.40;

    score = 0;

    if real(m.lambda) >= 0
        score = score + W * 1e6;
        return
    end

    if m.zeta < zeta_min
        score = score + W * 2000 * (zeta_min - m.zeta)^2;
    end

    if (m.zeta * m.wn) < zetaWn_min
        score = score + W * 2000 * (zetaWn_min - m.zeta * m.wn)^2;
    end

    if m.wn < wn_min
        score = score + W * 1200 * (wn_min - m.wn)^2;
    end
end

function score = score_roll_mode(m, p)
    % Want fast roll subsidence.
    % A smaller tau is better.
    % Penalize unstable roll mode heavily.

    if isfield(p, 'w_rollSubsidence')
        W = p.w_rollSubsidence;
    else
        W = 6;
    end

    tau_max = 1.40;
    tau_tgt = 0.50;

    score = 0;

    if real(m.lambda) >= 0
        score = score + W * 1e6;
        return
    end

    if m.tau > tau_max
        score = score + W * 800 * (m.tau - tau_max)^2;
    else
        score = score + W * 20 * (m.tau - tau_tgt)^2;
    end
end

function score = score_spiral(m, p)
    % Spiral can be lightly unstable, but should diverge slowly.
    % Penalize fast divergence strongly.

    if isfield(p, 'w_spiral')
        W = p.w_spiral;
    else
        W = 8;
    end

    tDouble_min = 20.0;

    score = 0;

    if real(m.lambda) > 0
        if isfinite(m.tDouble)
            if m.tDouble < tDouble_min
                score = score + W * 1200 * (tDouble_min - m.tDouble)^2;
            end
        else
            score = score + W * 1e5;
        end
    else
        % Stable spiral is okay. Do not overweight it.
        score = score + W * 0.0;
    end
end