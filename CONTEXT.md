# LCTK Calibration

LCTK calibration relates synchronized sensor observations to an extrinsic estimate and reports how
well the captured geometry constrains that estimate.

## Language

**Calibration Target**:
The physical artifact observed by the sensors: one plate, its fiducial layout, its canonical frame,
and the identity binding those facts together.
_Avoid_: Marker, hollow board (for the general concept), ArUco board

**Target Definition**:
The immutable description of one Calibration Target's physical geometry and fiducial layout. It
excludes sensor-specific detection settings and deployment-specific cropping.
_Avoid_: Board config, ArUco config, detector config

**Target Identity**:
The versioned identity of a Target Definition, used to prevent observations or archives from one
Calibration Target being interpreted as another.
_Avoid_: Frame convention, target name, config path

**Detector Tuning**:
Sensor- and operating-range-specific settings controlling how observations of a Calibration Target
are found and accepted.
_Avoid_: Target Definition, board geometry

**Camera-Frame Board Pose**:
The board pose in camera coordinates recovered from target marker geometry and image corners for an extrinsic solve. The LiDAR-to-camera solver owns this solve; camera marker detection and LiDAR target detection provide its observations.
_Avoid_: ArUco ICP pose, LiDAR board pose, extrinsic transform

**ICP Termination**:
The condition that ends an iterative pose-refinement attempt. A Good-Fit Termination and a Stable-Pose Termination are successful outcomes; an Iteration-Limit Termination or invalid state is unsuccessful.
_Avoid_: Final acceptance, convergence status

**Good-Fit Termination**:
An ICP attempt ending because its current residual is below the configured good-fit threshold. It is a successful ICP outcome.
_Avoid_: Final acceptance gate, rejection threshold

**Stable-Pose Termination**:
An ICP attempt ending because successive pose updates remain stable for the required stability window. It is successful even when the residual remains above the good-fit threshold.
_Avoid_: Good fit, iteration limit

**Iteration-Limit Termination**:
An ICP attempt reaching its configured iteration limit without a Good-Fit or Stable-Pose Termination. It is an unsuccessful ICP outcome.
_Avoid_: Max-iteration success, final acceptance

**Structural Evidence**:
Target-specific geometric support that distinguishes the winning pose from competing hypotheses. It includes sufficient inlier support, a clear loss advantage over alternatives, and cutout-rim support; it is separate from residual quality.
_Avoid_: Good fit, convergence, raw point count

**LiDAR Orientation Reference**:
The physical evidence that identifies a Calibration Target's named in-plane axes to a LiDAR: either
asymmetric cutouts or a required local axis aligned with mounting-up.
_Avoid_: Initial rotation, ICP orientation, camera marker orientation

**Transport Reliability**:
The reliability a sensor topic is subscribed with, resolved per device from what the session states
or what its recording offers. It is a property of whatever publishes the topic, not of the graph;
LCTK's own detection and transform topics are pinned and take no part in it.
_Avoid_: Mode, offline, realtime, QoS profile

**Detection Pair**:
One genuinely simultaneous camera ArUco detection and LiDAR board detection describing the same
observation instant.
_Avoid_: Synchronized group, frame pair

**Capture**:
A Detection Pair deliberately retained in the Detection Buffer. Multiple captures may describe one
Board Placement.
_Avoid_: Pose, placement, frame

**Board Placement**:
One geometrically distinct board position and plane orientation. Moving the board or tilting its
plane creates a new placement; repeated observations and in-plane rotation do not.
_Avoid_: Pose, capture, frame

**Detection Buffer**:
The ordered collection of synchronized LiDAR-camera detection pairs for one calibration, together
with the estimate and quality derived from exactly that collection.
_Avoid_: Detection cache, frame list, calibration session

**Solved Estimate**:
The extrinsic estimate derived from the current Detection Buffer revision. An estimate derived from
earlier buffer contents is stale and ineligible for publication.
_Avoid_: Last transform, cached transform, current transform

**Quality Verdict**:
The assessment of how well captured board geometry constrains a Solved Estimate. A degenerate
verdict does not mean the numerical solve failed.
_Avoid_: Solve status, calibration success

**Adjusted Transform**:
A publishable transform obtained by manually editing the current Solved Estimate. It is anchored to
that estimate and is cleared or rebased when the Detection Buffer changes.
_Avoid_: Solved estimate, calibration result

**Detection Archive**:
A versioned saved representation of a Detection Buffer, its Quality Verdict, and an optional
Adjusted Transform.
_Avoid_: Dump file, saved buffer
