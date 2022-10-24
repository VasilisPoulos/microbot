# PID observations and value tuning

## Tuning P value

At first p value was too low, the controller never reached
the set point (slow response speed).

Setting the p value to 0.0000093 the system responded quickly
but the system now had big overshoots on the way up and
sometimes on the way down.

![img](/images/physics%20stability%20testing/new_ranges/low_level_controller_kp_high.png)

I think the p value is too high, so i will try to find a
lower p value that works better.

Left it at 0.000009, not too many spikes. I think i need to add
an I value but i couldn't figure out how the i affects the system.
