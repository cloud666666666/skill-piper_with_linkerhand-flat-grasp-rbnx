# skill-piper-linker-flat-grasp-rbnx

Robonix skill for **flat-palm enveloping grasp** with an AgileX Piper arm and a
LinkerHand O6 dexterous hand. Owns `robonix/skill/pick/*`.

Catalog name: `robonix.skill.pick.flat_grasp`.

The grasp is *flat* — the palm is held roughly parallel to the table and the four
fingers curl around the object from the side — rather than a top-down gripper
closing on it. That posture is why the wrist has to be pinned during IK and why
the wrist joint angles are taught rather than solved; see the two upstream
services below.

## Capability surface

| Contract | Transport | Purpose |
| --- | --- | --- |
| `robonix/skill/pick/driver` | gRPC | Lifecycle (`CMD_INIT`; `CMD_ACTIVATE` is lazy, on first MCP call) |
| `robonix/skill/pick/grab` | MCP | Grasp the named object and **keep holding** it |
| `robonix/skill/pick/pick` | MCP | Grasp it, put it down at the configured place point, park |
| `robonix/skill/pick/put_down` | MCP | Release a held object at the point it was grasped from |
| `robonix/skill/pick/home` | MCP | Move to a resting pose, optionally de-energising the arm |

`CAPABILITY.md` in this repo is written for the calling LLM — it describes when
to reach for each tool and how to read the failure prefixes. Read that one if you
are wiring this into an agent; read this one if you are deploying it.

## What it orchestrates

Every endpoint is resolved through atlas — no hardcoded topic names, URLs or
ports:

```
detect_object    mcp   service/perception/object_detect/detect_object
grasp_request    grpc  service/perception/grasp_pose/grasp_request
execute_grasp    grpc  service/manipulation/execute_grasp
teach_safe       grpc  service/manipulation/teach_safe     (optional)
hand_move        grpc  primitive/hand/move_joint
hand_state       grpc  primitive/hand/get_state
```

The skill exists because each of those covers exactly one layer, and the grasp
**sequence** — open the hand, approach above the object, descend, close the
fingers, verify, lift, and for `pick` reverse it — is not any single service's
job. `execute_grasp` moves the arm and nothing else; the hand is a separate
primitive whose commands are rpc, so somebody has to own the ordering.

The flat-palm geometry itself is **not** here: it lives in `grasp_pose`'s flat
mode (pixel → arm pose, including the heading rule) and in `roboarm_ik`'s
`solve_flat` (pose → joints, with the wrist pinned).

## Runtime shape

```
grab(object_name)
  detect_object                      -> bbox (YOLO OBB)
  grasp_pose.grasp_request           -> xyz + orientation + object width
  hand_move (open palm)
  execute_grasp  (approach, +offset) -> arm
  execute_grasp  (descend)
  hand_move (close)
  get_state                          -> verify the fingers met something
  execute_grasp  (lift)              -> returns while still holding

pick(object_name)
  ... same as grab ...
  execute_grasp  (to the place point)
  hand_move (open palm)
  execute_grasp  (retreat)
  teach_safe
```

### Grasp verification

`grab` does not report success just because the motion sequence finished.
Robonix's own `Pick.srv` defines `success` as "the arm completed the grasp motion
**and** fresh gripper feedback confirmed that an object is held", so after
closing, the skill reads the hand's axis positions and refuses to report success
if the four finger axes reached the fully-closed command — that means the fingers
closed on air. A miss is otherwise indistinguishable from a grasp: the arm looks
right and every step reported OK.

Measured on real hardware (contract units): holding a tomato the fingers stop
around `0.42–0.49`; closing on air they reach `1.0`. The gap is ~0.5, so the
default tolerance of `0.12` is not a fine judgement. Tune with
`grasp_verify_tolerance`.

### Where objects are placed

`put_down` releases an object at the point it was grasped from. `pick` places it
at the class's configured `place_pos` entry, falling back to the grasp point when
the class has no entry — so an unconfigured deploy always returns objects exactly
where they were, and can never drop one somewhere unintended.

```yaml
place_pos:
  tomato: {pos: [0.30, -0.10]}   # literal metres in arm/base_link
  potato: {pos: ["x", "-y"]}     # relative to the object's own position
```

A `pos` element may be a number, or one of `x` / `-x` / `y` / `-y` / `z` / `-z`,
which substitute the object's *current* grasp coordinate. Giving `[x, y]`
inherits the grasp height for z — do not lower it, or the hand presses into the
table. `place_distance_threshold_m` skips the whole cycle when the object is
already within that distance of its place point.

## Config (via `skill[].config` in the deploy manifest)

| key | default | meaning |
| --- | --- | --- |
| `hand_joint_names` | the six global axis names | Fixed by the hand contract set; override only for a different hand. |
| `hand_open_pose` | `[0.0, 0.725, 0.0, 0.0, 0.0, 0.0]` | Open-palm pose, contract units `[0,1]`, `0=open / 1=closed`. |
| `hand_close_pose` | `[0.6, 0.929, 1.0, 1.0, 1.0, 1.0]` | Fist pose, contract units. |
| `grasp_verify_tolerance` | `0.12` | Max per-axis gap that still counts as "closed on air". |
| `safe_z_offset_m` | `0.05` | Approach/retreat height above the grasp point. **Not free to raise** — holding the palm flat costs wrist range, which grows with height. The skill walks this down automatically if the IK refuses. |
| `motion_timeout_s` | `25.0` | Per-motion budget handed to `execute_grasp`. |
| `settle_s` | `1.5` | Pause after a move, before touching the hand. |
| `close_settle_s` | `2.0` | Wait for the fingers to finish closing before verifying. |
| `park_after_pick` | `true` | `teach_safe` once the object is down. |
| `place_pos` | `{}` | Per-class place points (see above). |
| `place_heading_offset_deg` | `4.2` | Must equal `grasp_pose`'s `heading_offset_deg`. The place point has its own bearing, so the wrist is re-aimed at it. |

Poses are converted from the firmware's `0–255` by `1 - sdk/255` — **inverted**,
because the firmware uses `255` = fully open while the contract uses `1` =
closed.

## Build

```bash
rbnx build -p .
```

No colcon step: this package ships no ROS source and vendors no ROS tree. Its
IDLs are package-local (`capabilities/lib/`) plus the hand contracts it consumes,
which come from the robonix source tree.

## Known limits

* The place point must be within the arm's reach **and reachable while holding
  the palm flat**. Close-in objects are the hard case: on this arm an object at
  x≈0.22 m has IK cost 3.5 at z=0.11, 0.2 at z=0.16, 8.0 at z=0.21 (the flat
  solver accepts ≤4.0). That is what `safe_z_offset_m`'s automatic walk-down is
  for.
* Object poses come from a 2D homography calibration, so accuracy is
  **per-point**: a few centimetres of table position can change a grasp from
  hit to miss. Re-calibrate before moving the working area.
