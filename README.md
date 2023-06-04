# grblhal_probe_plugin
Plugin for probing and probe protection in GRBLHAL. 

Features:
- Configure probe polarity independently for tool probe and touch probe.  Allows easy disconnection of NC probes.
- On PROBE_CONNECTED check probe pin and assert halt outside of any movement that isn't a probing motion.
- Allow PROBE_CONNECTED to be assigned to Aux input (set polarity)
- Set PROBE_CONNECTED on T99.

In future:
- Set jog exclusion zone around toolsetter.
