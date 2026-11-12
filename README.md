# DH to URDF

Convert a standard or modified Denavit–Hartenberg table into a connected URDF chain.
Open [the converter](https://adohaha.github.io/DH2URDF/) or open `index.html` locally.
The browser generator has no runtime dependencies. `dh-converter.js` is its source.

## Input

Standard DH uses `|θᵢ|dᵢ|aᵢ|αᵢ|joint|` (ASCII:
`|theta_i|d_i|a_i|alpha_i|joint|`), representing
θᵢ, dᵢ, aᵢ, αᵢ. Modified DH uses
`|aᵢ₋₁|αᵢ₋₁|θᵢ|dᵢ|joint|` (ASCII:
`|a_{i-1}|alpha_{i-1}|theta_i|d_i|joint|`), representing
aᵢ₋₁, αᵢ₋₁, θᵢ, dᵢ. The older short headers `th,d,a,alpha,R`
and `a,alpha,th,d,R` are also accepted. A Markdown separator row is optional. Angles are
radians and lengths are metres. Numerical cells allow `pi`, parentheses,
and `+ - * /`; for example, `-pi/2`. The last column accepts
`true`/`false`, `R`/`P`, or `revolute`/`prismatic`.

For a revolute row, the joint variable adds to θᵢ. For a prismatic row,
it adds to dᵢ. The generated URDF uses two joints per row to preserve the
order of transforms:

| Convention | Transform |
| --- | --- |
| Standard | `Rz(θᵢ + qᵢ) Tz(dᵢ) Tx(aᵢ) Rx(αᵢ)` (revolute); `Rz(θᵢ) Tz(dᵢ + qᵢ) Tx(aᵢ) Rx(αᵢ)` (prismatic) |
| Modified | `Tx(aᵢ₋₁) Rx(αᵢ₋₁) Rz(θᵢ + qᵢ) Tz(dᵢ)` (revolute); `Tx(aᵢ₋₁) Rx(αᵢ₋₁) Rz(θᵢ) Tz(dᵢ + qᵢ)` (prismatic) |

`Tx(aᵢ)` and `Rx(αᵢ)` commute, as do `Rz(θᵢ)` and `Tz(dᵢ)`.
The base and each row's final link show red X, green Y, and blue Z frame
markers, following [RViz's default axis colors](https://github.com/ros2/rviz/blob/rolling/rviz_rendering/src/rviz_rendering/objects/axes.cpp).
The URDF is a kinematic scaffold: edit link visuals, collision geometry,
inertial data, and physical joint limits for your robot. Generated slider
ranges are ±π radians for revolute joints and ±1 metre for prismatic joints.
The effort and velocity values are placeholders.

Use **Download .urdf** and load the file under **URDF and STL files** in
[MyModelRobot](https://mymodelrobot.appspot.com/), or use **Copy URDF** and
paste it into its URDF editor.

## Verification

```sh
python3 -m unittest discover -s tests -v
lean verification/DH.lean
```

The Python checks parse real generator output as XML, verify a single connected
tree, and compare joint transforms numerically with standard and modified DH
at nonzero revolute and prismatic positions. The Lean proof establishes the
transform ordering for arbitrary associative composition under the z-axis commutation
relations; it does not prove the JavaScript parser or XML serializer correct.
