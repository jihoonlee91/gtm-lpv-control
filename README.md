# LPV Control

Designs a gain-scheduled (LPV, linear parameter-varying) autopilot for a variable span-sweep morphing aircraft, in the GTM (Generic Transport Model) simulation environment. Related paper: "Linear Parameter-Varying Control of Variable Span-Sweep Morphing Aircraft" (AIAA SciTech Forum, 2019, [DOI: 10.2514/6.2019-0106](https://doi.org/10.2514/6.2019-0106)).

## Approach

The aircraft's span/sweep morphing configuration changes its aerodynamics continuously, so a single linear controller designed at one configuration does not perform well across the full morphing envelope. This repository treats the morphing configuration as a scheduling parameter: the nonlinear aircraft is trimmed and linearized at a grid of configurations, longitudinal and lateral autopilots (and inner-loop control augmentation systems) are designed at each grid point using the LPVTools toolbox, and the resulting gains are scheduled online as the configuration varies.

## Repository Structure

- `GTM_DesignSim/`: NASA GTM_DesignSim-based nonlinear aircraft dynamics simulation (see its own `README.md`)
  - `gtm_design/design/`: `trim_gtm.m`, `linearize_gtm.m` — trim and linearization across the morphing grid; `design_lon_autopilot.m`, `design_lon_cas.m`, `design_lat_autopilot.m`, `design_lat_cas.m` — longitudinal/lateral autopilot and control augmentation system (CAS) design
  - `gtm_design/lpv_design.m`: top-level LPV controller design script
  - `gtm_design/misc/`: `tune_cas_lon.m`, `tune_cas_lat.m` — CAS gain tuning helpers
  - `gtm_design/plot/`: design/autopilot/CAS result plots, including `plot_morphing_effect.m` and `trim_map.fig`
  - `gtm_design/sfunction/`: S-function blocks used inside Simulink (rigid-body dynamics, NDI, control-augmentation variants)
  - `gtm_design/data/`: saved trim/linearization results (`lpv_design_prep.mat`, `lpv_design_result.mat`) and the FixedWingBus definition
  - `gtm_design/trajectory3/`: reference trajectory data and aircraft silhouette models used for 3D visualization of simulated flight paths
  - `gtm_design/LPVToolsV1.0/`: LPVTools toolbox (MUSYN group, Univ. of Minnesota) used for the LPV design

## How to Run

Open MATLAB, `cd` into `GTM_DesignSim/gtm_design`, run `setup.m` to add the required paths (including LPVTools), then run `lpv_design.m` to trim/linearize the aircraft across the morphing grid and synthesize the scheduled controllers.

Development environment: MATLAB

## Citation

```bibtex
@inproceedings{lee2019lpv,
  title     = {Linear Parameter-Varying Control of Variable Span-Sweep Morphing Aircraft},
  author    = {Lee, Jihoon and Kim, Seong-hun and Jung, Seungyun and Lee, Hanna and Kim, Youdan},
  booktitle = {AIAA Scitech 2019 Forum},
  address   = {San Diego, CA},
  year      = {2019},
  doi       = {10.2514/6.2019-0106}
}
```

Part of my morphing-aircraft control research — more at [jihoonlee91.github.io](https://jihoonlee91.github.io/).
