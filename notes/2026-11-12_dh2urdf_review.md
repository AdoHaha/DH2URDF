# DH to URDF generator review

The previous standard DH template referenced a passive link it did not emit.
The replacement creates a connected chain with one movable and one fixed joint
per DH row. This is the general two-joint decomposition needed to preserve the
standard and modified DH transform order, including nonzero offsets.

The generator uses the classic indexed DH labels in the UI and accepts ASCII
and legacy headers. It validates all rows before producing output, rejects
nonfinite expressions, and offers copy and download actions for MyModelRobot.
The generator is now plain JavaScript with no browser libraries; the former
CoffeeScript source and unused bundled libraries were removed.

Verification on 2026-11-12:

- Python tests parsed generated XML, checked unique links and a connected
  acyclic joint tree, and compared the generated transforms with independent
  4×4 DH matrix calculations for standard and modified conventions with
  revolute and prismatic joints at nonzero positions.
- Lean 4 compiled proofs of the four row-order cases and arbitrary chain
  composition without `sorry`.
- A Chromium smoke check generated the five-row sample and imported it into
  the local MyModelRobot app. The app loaded 10 joints, 11 links, and two frame
  visuals on the final link without page errors.

Scope: the Lean proof covers algebraic ordering. It does not verify the
JavaScript parser or serializer, which are checked by the executable tests.
Generated joint ranges and effort/velocity are editing placeholders rather
than physical specifications.
