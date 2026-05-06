# Jäger — Firefighter VTOL Drone

Conceptual design of an autonomous VTOL fixed-wing drone for wildfire suppression, developed as a 6-person team in MAE 155A at UC San Diego.

The drone is sized to deliver two fire suppression munitions to a target 1.5 miles away within 180 seconds of takeoff, operating under FAA Part 107 altitude restrictions (400 ft AGL). The design uses a hybrid VTOL architecture — multi-rotor hover for takeoff and payload delivery, fixed-wing cruise for range efficiency.

![Jäger deploying fire suppression munitions](images/jager_render_hero.png)

---

## Mission Profile

The mission is time-critical: from launch to first payload release in under 3 minutes.

| Phase | Duration | Altitude |
|---|---|---|
| Takeoff & climb | 30 s | 0 → 400 ft |
| Outbound cruise | 65 s | 400 ft |
| Target acquisition & descent | 45 s | 400 → 25 ft |
| Payload delivery (×2) | 40 s/munition | 25 ft |
| Recovery & return | 38 s | 25 → 400 ft |

![Mission Profile](CoDR/Improve%20Matlab%20Figures/missionprofiles.png)

---

## Design

| Front view | Side profile |
|---|---|
| ![Front](images/jager_render_front.png) | ![Side](images/jager_render_side.png) |

## Key Design Parameters

| Parameter | Value |
|---|---|
| Gross weight | 55 lb (25 kg) |
| Payload | 2 × fire suppression munitions (3.3 lb each) |
| Cruise speed | ~51 mph (75 ft/s) |
| Range | 1.5 miles (one-way) |
| Max altitude | 400 ft AGL (FAA Part 107) |
| Drop altitude | 25 ft AGL |
| Wing airfoil | NACA 4412 |
| Aspect ratio | 6 |
| Wing area | 6.64 ft² |
| Wingspan | 6.31 ft |
| Disk loading | 250 N/m² |

---

## Aerodynamic Analysis

**Airfoil — NACA 4412**

![NACA 4412](CoDR/Improve%20Matlab%20Figures/NACA4412.png)

**Lift curve and drag polar**

![CL vs Alpha](CoDR/Improve%20Matlab%20Figures/CL%20vs%20alpha.png)
![CL vs CD Polar](CoDR/Improve%20Matlab%20Figures/CL%20vs%20CD%20polar.png)

---

## Sizing and Constraints

Power-to-weight vs wing loading sizing diagram showing governing constraints (stall, climb, maneuver, hover).

![Sizing Constraint](CoDR/Improve%20Matlab%20Figures/Sizing_Constraint.png)

---

## Stability

**Static margin** computed from CG and aerodynamic center locations across the fuselage axis.

![Static Margin](CoDR/Improve%20Matlab%20Figures/StaticMargin.png)

**V-n diagram** including gust loads at cruise and dive speeds.

![V-n Diagram](CoDR/Improve%20Matlab%20Figures/VNDiagram_final.png)

---

## MATLAB Analysis Code

| Script | Description |
|---|---|
| `VTOL_SizingV9Mk2.m` | Full VTOL sizing — weight, battery fractions, hover/cruise power, mission timing |
| `JagerAeroAnalysis.m` | Aerodynamic analysis — CL-alpha curve, drag polar, parasite drag build-up from CAD wetted areas |
| `ParasiteDragV1.m` | Component-level parasite drag estimation |
| `CrusieSpeedCalc.m` | Cruise speed requirement from mission time budget |
| `Range_and_Weight.m` | Range and weight fraction trade |
| `Dynamic Stability Analysis/main_vtailSweep_JAGER.m` | AVL-based V-tail geometry sweep for dynamic stability |
| `Dynamic Stability Analysis/OptomizedDynamicsStabillityAnalysisV2.m` | Optimized dynamic stability analysis |
| `Control/JAGER_AVL_AUTORUN_CONTROL_SIZING_V8_OFFTIP_SPREADSHEET.m` | Control surface sizing via AVL automation |

---

## Documents

- `CoDR/CoDR Jager-V5.pptx` — Full Conceptual Design Review presentation
- `Weight & Balance Master.xlsx` — Component-level weight and balance breakdown
- `CAD files/Drawings/` — Assembly drawings (PDF)

---

## Team

6-person team, MAE 155A — Aircraft Design, UC San Diego (Winter 2026).
Juan Manuel Sanchez contributed aerodynamic analysis, MATLAB sizing code, and dynamic stability analysis.
