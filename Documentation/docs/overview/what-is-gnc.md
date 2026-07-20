---
title: "What Is Guidance, Navigation, and Control?"
slug: "/overview/what-is-gnc/"
---

Guidance, Navigation, and Control (GNC) is a discipline that combines mathematics, physics, dynamics, modelling, optimization, software and data analysis. Put simply:

- **Guidance**

  Where should I be going?

- **Navigation**

  Where am I?

- **Control**

  OK, go this way.

Or, thought of as a classic control loop, Guidance provides the setpoint, Navigation provides the "plant" measurements (current state), and Control is the algorithm that brings the measurements to the setpoint by adjusting an actuator (providing the control "input"). Obviously it gets more complicated than this once you factor in your setpoint might be an orbit around the Earth, your measurements are noisy and don't provide exactly what you need, and your control system has multiple actuators in multiple axes! But with the right combination of mathematics, modelling and physics a GNC engineer can produce software and algorithms that can launch a rocket into orbit, or a place a spacecraft around the moon.

See [What Does a GNC Engineer Do?](./what-gnc-engineers-do.md) for more details on what the day-to-day tasks of a GNC engineer look like.
