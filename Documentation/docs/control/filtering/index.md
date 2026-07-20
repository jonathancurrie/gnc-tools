---
title: "Digital Filtering"
slug: "/control/filtering/"
---

This section will cover digital (discrete) filter theory, design and practical tips on application.

- **FFT**

  Learn how to apply a Fast Fourier Transform and understand its output.

- **[Aliasing](./aliasing.md)**

  Understand aliasing and how to avoid it.

- **FIR vs IIR**

  Learn about Finite and Infinite Impulse Response Filters.

- **Second Order Sections**

  Understand the numerical benefits of Second Order Section Filters.

- **[Software](./software.md)**

  Digital Filter C/C++ Implementation Example.

- **[Design](./filter-design.md)**

  Learn how to design digital filters using MATLAB.

- **Practical Considerations**

  Tips and Tricks on Implementing Filters.

Clone [GNCTools](https://github.com/jonathancurrie/gnc-tools) to access the examples and source code for the above links. Example code can be found in `Control/Filter/Filter_Examples.m`.

## Nomenclature

The following diagram shows the format of the GNCTools digital filter, together with variable naming.

![Filter Block Diagram](/img/gnc/FilterBlockDiagram.png)

|  |  |
| --- | --- |
| **Variable** | **GNC Tools Name** |
| `u` | Input |
| `y` | Output |
