![Logo](/readme_images/logo_sm.jpg)
# GRBLHAL Probing Add-on Plugin

Features:
- Configure probe polarity independently for tool probe and touch probe.  Allows easy disconnection of NC probes.
- On PROBE_CONNECTED check probe pin and assert halt outside of any movement that isn't a probing motion.
- On PROBE_CONNECTED do not allow the spindle to run.
- Allow PROBE_CONNECTED to be assigned to Aux input (set polarity)
- Set PROBE_CONNECTED on T99.
- Set PROBE_CONNECTED with M401 and clear with M402 mcodes.
- Allow hard limits to be enabled during tool probe.

In future:
- Set jog exclusion zone around toolsetter.
