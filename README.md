# LPV Control

Repository for designing an LPV (linear parameter-varying) controller in the GTM (Generic Transport Model) simulation environment, related to "Linear Parameter-Varying Control of Variable Span-Sweep Morphing Aircraft" (AIAA SciTech Forum, 2019).

**Content:** Designs a gain-scheduled controller that treats the aircraft's span/sweep morphing configuration as a varying parameter, using the LPVTools toolbox on top of a GTM_DesignSim-based nonlinear aircraft model.

- `GTM_DesignSim/`: NASA GTM_DesignSim-based nonlinear aircraft dynamics simulation
- `GTM_DesignSim/gtm_design/lpv_design.m`: LPV controller design script
- `GTM_DesignSim/gtm_design/LPVToolsV1.0/`: LPVTools toolbox used for the LPV design

Development environment: MATLAB
