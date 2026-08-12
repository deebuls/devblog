---
categories:
- robotics
date: '2026-08-12'
description: A explanation of how theoretical formulas has to be changed as per realistic sensor data
layout: post
title: Theory vs. Practice in 3D Point Transformation
toc: true

---
# Theory vs. Practice in 3D Point Transformation

*How Small Measurement Errors Cause Large Calculation Errors in Real World Applications*

I had to implement a simple Horizontal coordinate(elevation,azimuth,distance) to cartesian cordinate(x,y,z) convertor . Which is a simple matrix multiplication.
I implemented it and was working perfectly in most cases until it was not working in some cases. 
In further debugging you find series of problems on how a small error in some readings causes 10 times big error in the final output. 
This blog is a summary on how you can still remove these errors by putting some checks and conversion.

---

## 1. Introduction: Theory vs. Practice

Mathematical formulas work in clean environments. Perfect formulas assume perfect inputs. In theory, if you measure three points on room walls, pure geometry calculates exact 3D point transformations.

Real environments are not clean. Physical tools always have measurement noise. Human operators introduce small offsets. When real measurement errors go into theoretical formulas, the math amplifies these small errors into large calculation failures.

```mermaid
graph TD
    subgraph Theoretical_Model [Theoretical Model]
        A1[Perfect Point Measurements] --> B1[Pure Geometry Formula]
        B1 --> C1[Exact Point Output - 0 cm Error]
    end

    subgraph Real_Practice [Real World Practice]
        A2[Measurement Noise - 1 cm Height Offset] --> B2[Coupling in 3D Matrix]
        B2 --> C2[Amplified Point Drift - 10 cm Horizontal Error]
    end

    style Theoretical_Model fill:#eef2f7,stroke:#1d3557,stroke-width:2px
    style Real_Practice fill:#fdf2f2,stroke:#e63946,stroke-width:2px
    style C1 fill:#1d3557,color:#ffffff
    style C2 fill:#e63946,color:#ffffff

```

---

## 2. The Original Solution: Real-World Error Analysis

The original process converts spherical point coordinates $(r, \theta, \phi)$ to 3D Cartesian coordinates $(x, y, z)$. It uses three measurement points:

* **Point A & Point B:** Measured on Wall 1 to establish the baseline orientation.
* **Point C:** Measured on a perpendicular wall to establish the room alignment.

```mermaid
graph LR
    subgraph Spherical_Input [Spherical Inputs]
        S1[Radius: r]
        S2[Azimuth Angle: θ]
        S3[Zenith Angle: φ]
    end

    subgraph Cartesian_Convert [Cartesian Conversion]
        C1["x = r * sin(φ) * cos(θ)"]
        C2["y = r * sin(φ) * sin(θ)"]
        C3["z = r * cos(φ)"]
    end

    subgraph Matrix_Transform [3D Frame Transformation]
        M1["u_x = normalize(p2 - p1)"]
        M2["u_y = normalize(global_z × u_x)"]
        M3["Translation Vector = -dot(u, origin)"]
    end

    S1 & S2 & S3 --> Cartesian_Convert
    Cartesian_Convert --> Matrix_Transform

    style Spherical_Input fill:#f8f9fa,stroke:#457b9d
    style Cartesian_Convert fill:#eef2f7,stroke:#1d3557
    style Matrix_Transform fill:#fff3bf,stroke:#fab005

```

Here is the expanded **Original Solution: Matrix Generation & Real-World Error Analysis** section with detailed text and diagrams explaining how the 3D transformation matrix is constructed and why real-world measurement noise breaks it.

---

## 2. The Original Solution: Matrix Generation & Error Analysis

The original system converts spherical point measurements $(r, \text{azimuth}, \text{zenith})$ into Cartesian 3D coordinates $(x, y, z)$. It uses three reference targets:

* **Point A & Point B:** Measured along Wall 1 to define the main horizontal orientation axis.


* **Point C:** Measured on a perpendicular wall to locate the room origin.



### Step-by-Step 3D Transformation Matrix Generation

```mermaid
graph TD
    subgraph Step_1 [1. Spherical to 3D Cartesian Conversion]
        A["Point Inputs (r, azimuth, zenith)"] --> B["x = r * sin(zenith) * cos(azimuth)"]
        A --> C["y = r * sin(zenith) * sin(azimuth)"]
        A --> D["z = r * cos(zenith)"]
    end

    subgraph Step_2 [2. Establish Basis Axis Vectors]
        E["Primary Baseline Vector: p2 - p1"] --> F["X-Axis Unit Vector: u_x = normalize(p2 - p1)"]
        F --> G["Y-Axis Unit Vector: u_y = normalize(global_z × u_x)"]
        G --> H["Z-Axis Unit Vector: u_z = cross(u_x, u_y)"]
    end

    subgraph Step_3 [3. Calculate 3D Translation & Assemble Matrix]
        H --> I["Intersection Point (x_int, y_int, -h_device)"]
        I --> J["T_x = -dot(u_x, origin_new)"]
        I --> K["T_y = -dot(u_y, origin_new)"]
        J & K --> L["Assemble 4x4 Homogeneous Transformation Matrix"]
    end

    style Step_1 fill:#eef2f7,stroke:#457b9d
    style Step_2 fill:#eef2f7,stroke:#457b9d
    style Step_3 fill:#fff3bf,stroke:#fab005
    style L fill:#1d3557,color:#ffffff

```

To align all measured points with the room geometry, the original math constructs a **4x4 Homogeneous Transformation Matrix** using the following steps:

1. **Vector Orientation ($u_x, u_y, u_z$):**
The primary axis $u_x$ is derived directly by subtracting 3D Point A from 3D Point B. $u_y$ is created using the cross product of the global Z vector $(0, 0, 1)$ and $u_x$. $u_z$ is the cross product of $u_x$ and $u_y$.


2. **Intersection Point ($x_{\text{int}}, y_{\text{int}}, -h_{\text{device}}$):**
The math projects lines from $p_1 \to p_2$ and from $p_3$ perpendicularly to find where wall alignment lines intersect on the floor plane. The device height ($h_{\text{device}}$) is applied to set $z_{\text{origin}} = -h_{\text{device}}$.


3. **3D Dot Product Translation Matrix:**
Translations $T_x$ and $T_y$ are calculated using full 3D dot products between the direction unit vectors ($u_x, u_y$) and the origin vector

$$T_x = -(u_{x.x} \cdot x_{\text{int}} + u_{x.y} \cdot y_{\text{int}} + u_{x.z} \cdot z_{\text{int}})$$

$$T_y = -(u_{y.x} \cdot x_{\text{int}} + u_{y.y} \cdot y_{\text{int}} + u_{y.z} \cdot z_{\text{int}})$$

$$\text{Original Matrix} = \begin{bmatrix}  u_{x.x} & u_{x.y} & u_{x.z} & T_x \\  u_{y.x} & u_{y.y} & u_{y.z} & T_y \\  0 & 0 & 1 & h_{\text{device}} \\  0 & 0 & 0 & 1  \end{bmatrix}$$


---

### Problem 1: Height Offset Leakage into Horizontal Coordinates

The device height ($h_{\text{device}}$) sets the Z-axis origin position: $z_{\text{origin}} = -h_{\text{device}}$. The original mathematical formula projects full 3D vector dot products into X and Y coordinate translations:

$$T_x = -(u_{x.x} \cdot x_{\text{int}} + u_{x.y} \cdot y_{\text{int}} + u_{x.z} \cdot z_{\text{int}})$$

$$T_y = -(u_{y.x} \cdot x_{\text{int}} + u_{y.y} \cdot y_{\text{int}} + u_{y.z} \cdot z_{\text{int}})$$

When Wall 1 points are not perfectly level, $u_{x.z}$ is not zero. Therefore, $z_{\text{origin}}$ directly changes the horizontal $T_x$ and $T_y$ position values.

```mermaid
graph TD
    A[Device Height Offset: Δz = 1 cm] --> B[Z Origin Component: z_int = -h_device]
    B --> C["Tilted Vector Axis: u_x.z ≠ 0"]
    C --> D["Translation Calculation: T_x = -(u_x.x*x + u_x.y*y + u_x.z*z)"]
    D --> E[Horizontal Shift: ΔX = 10 cm Error]

    style A fill:#ffe3e3,stroke:#e63946
    style C fill:#ffe3e3,stroke:#e63946
    style E fill:#e63946,color:#ffffff

```

---

### Problem 2: Angle Amplification over Distance

A measurement tool calculates points using radius distance $r$ and zenith angle $\phi$:

$$z = r \cdot \cos(\phi) \quad \text{and} \quad d_{\text{horizontal}} = r \cdot \sin(\phi)$$

When a target point is far from the device (for example, $r = 10\text{ m}$), a small vertical error changes the effective zenith angle. The long distance acts as a leverage arm, amplifying a 1 cm height error into a 10 cm horizontal error.

```mermaid
graph LR
    P1[Measurement Device] -- "Distance r = 10 m" --> P2[True Target Location]
    P1 -- "Height Error Δz = 1 cm" --> P3[Shifted Target Location]
    P2 -- "Horizontal Drift ΔX = 10 cm" --> P3

    style P1 fill:#1d3557,color:#ffffff
    style P2 fill:#2b9348,color:#ffffff
    style P3 fill:#e63946,color:#ffffff

```

---

## 3. Measurement Sensitivity Table

| Measurement Input | Real World Error | Primary Affected Matrix Axis | Final Output Error |
| --- | --- | --- | --- |
| **Device Height ($h_{\text{device}}$)** | $\pm 1.0\text{ cm}$ offset | Z-axis coupled into X-Y Translation | **$10.0\text{ cm}$ point position drift** |
| **Point A / B Zenith Angle** | $\pm 0.25^\circ$ sensor tilt | $u_{x.z}$ vertical vector component | $4.5\text{ cm}$ tilt error over $5\text{ m}$ distance |
| **Point A to B Baseline Distance** | Points closer than $0.5\text{ m}$ | $u_x$ orientation unit vector | Severe angular drift across entire room |

---

## 4. The Robust Updated Solution

The updated solution decouples vertical height calculations from horizontal calculations using a pure 2D planar projection before constructing the transformation matrix.

```mermaid
graph TD
    subgraph Step_1 [Step 1: Flatten Points]
        S1["Convert Spherical to 3D: (x, y, z)"] --> S2["Project to 2D Floor Plane: (x, y)"]
    end

    subgraph Step_2 [Step 2: Pure 2D Orientation]
        S2 --> S3["Calculate 2D Basis Vectors: u_x2d, u_y2d"]
        S3 --> S4["Set u_z Components to 0"]
    end

    subgraph Step_3 [Step 3: Isolated Height Application]
        S4 --> S5["Apply h_device ONLY to Matrix Row 3"]
        S5 --> S6["Final 4x4 Matrix with Zero Horizontal Leakage"]
    end

    style Step_1 fill:#eef2f7,stroke:#457b9d
    style Step_2 fill:#eef2f7,stroke:#457b9d
    style Step_3 fill:#d8f3dc,stroke:#2b9348
    style S6 fill:#2b9348,color:#ffffff

```

---

### How the Robust Solution Fixes the Problems

1. **Zero Horizontal Leakage:** The rotation vectors $u_{x2d}$ and $u_{y2d}$ have explicit Z values of zero ($u_z = 0$). Device height offset only affects the output Z axis.
2. **Baseline Validation Guard:** The system checks baseline distance: $d = \sqrt{dx^2 + dy^2}$. If $d < 0.5\text{ m}$, calculation stops to prevent extreme sensitivity.
3. **Singularity Protection:** The code checks vertical conditions ($dx \approx 0$) and horizontal conditions ($dy \approx 0$) directly to prevent division by zero.

---

## 5. Mathematical Matrix Comparison

### Original Matrix (Coupled Errors):

$$M_{\text{original}} = \begin{bmatrix} u_{x.x} & u_{x.y} & u_{x.z} & T_x(z) \\ u_{y.x} & u_{y.y} & u_{y.z} & T_y(z) \\ 0 & 0 & 1 & h_{\text{device}} \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### Robust Matrix (Decoupled Height):

$$M_{\text{robust}} = \begin{bmatrix} u_{x2d.x} & u_{x2d.y} & 0 & T_{x2d} \\ u_{y2d.x} & u_{y2d.y} & 0 & T_{y2d} \\ 0 & 0 & 1 & h_{\text{device}} \\ 0 & 0 & 0 & 1 \end{bmatrix}$$
