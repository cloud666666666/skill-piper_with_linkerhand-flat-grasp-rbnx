# Public deploy config for robonix.skill.pick.flat_grasp.
# Values below are the ones this deploy uses; the type/unit/constraint note
# above each key is the contract.
config:
  # ── grasp geometry ──────────────────────────────────────────────────────
  # float, metres, default: 0.05; must be >= 0.
  # Travel height above the grasp point on approach/retreat, and the height the
  # object is carried at. NOT free to raise: holding the palm parallel to the
  # table costs wrist range and that cost grows with height — measured on this
  # arm at x=0.223, flat-IK cost is 3.5 at z=0.11, 0.2 at z=0.16, 8.0 at z=0.21,
  # 17.8 at z=0.24 (the solver accepts <= 4.0). The skill walks this offset down
  # by itself (x0.75/0.5/0.25) when the IK refuses.
  safe_z_offset_m: 0.05

  # float list, degrees, length 3, default: [-142.249, 81.015, 149.475].
  # The taught flat-palm orientation, interpreted as scipy's EXTRINSIC zyx
  # (lowercase!). The uppercase "ZYX" is intrinsic and differs by 73.5 deg here.
  flat_euler_deg_zyx: [-142.249, 81.015, 149.475]

  # ── timing ──────────────────────────────────────────────────────────────
  # float, seconds; all must be > 0. Per-motion budget for the arm, and the
  # pause after a motion before the hand is touched.
  motion_timeout_s: 25.0
  settle_s: 1.5
  # Wait for the fingers to finish closing, and after opening before retreating.
  close_settle_s: 2.0
  release_settle_s: 1.5
  # float, seconds, default: 0.15. Gap between publishing a joint command and
  # reading the resulting state back, so the driver has seen the command.
  hand_publish_gap_s: 0.15

  # ── hand poses (CONTRACT UNITS) ─────────────────────────────────────────
  # 6 floats each, normalized [0,1] with 0 = open and 1 = closed, in the axis
  # order below. Conversion from the vendor's firmware units is
  # `contract = 1 - raw/255` — the firmware uses 255 = fully open, so pasting a
  # value out of the vendor tooling needs that inversion.
  hand_open_pose: [0.0, 0.725, 0.0, 0.0, 0.0, 0.0]
  hand_close_pose: [0.6, 0.929, 1.0, 1.0, 1.0, 1.0]

  # string list, length 6. Fixed by the global hand contract set (the hand
  # primitive's `info` rpc returns exactly these) — do not reorder.
  hand_joint_names:
    [thumb_cmc_pitch, thumb_cmc_yaw, index_mcp_pitch,
     middle_mcp_pitch, ring_mcp_pitch, pinky_mcp_pitch]

  # float, contract units, default: 0.12.
  # If the four finger axes reach the fully-closed command after a grasp, the
  # fingers closed on air and the grasp is reported as failed. Measured on this
  # robot: holding the object the fingers stop around 0.42-0.49, empty they
  # reach 1.0 — the gap is ~0.5, so this is not a fine judgement.
  grasp_verify_tolerance: 0.12

  # ── place targets ───────────────────────────────────────────────────────
  # mapping: detector class name -> {pos: [x, y]} in arm/base_link metres.
  # Each element is either a literal number or a symbolic reference to the
  # object's CURRENT position: "x"/"-x"/"y"/"-y". A class with no entry is put
  # back exactly where it was picked from.
  place_pos:
    tomato:
      pos: [0.25, 0.10]

  # float, metres, default: 0.0 (disabled). Skip the pick-and-place cycle when
  # the object is already within this distance of its place point.
  place_distance_threshold_m: 0.0

  # float, degrees, default: 4.2. Must equal grasp_pose's heading_offset_deg.
  # Duplicated because the place point has its own bearing, so the wrist has to
  # be re-aimed at it: heading = atan2(place_y, place_x) + this.
  place_heading_offset_deg: 4.2

  # ── lifecycle / timeouts ────────────────────────────────────────────────
  # float, seconds; all must be > 0.
  # detect: wait for the detector; grasp_request: for the pose service;
  # teach_safe: to reach the standby pose; resolve: for atlas to hand back all
  # upstream endpoints on first activation.
  detect_timeout_s: 30.0
  grasp_request_timeout_s: 10.0
  teach_safe_timeout_s: 25.0
  resolve_timeout_s: 60.0
  # float, seconds; budget for the hand's move_joint / get_state rpcs.
  hand_move_timeout_s: 5.0
  hand_state_timeout_s: 5.0

  # boolean, default: true. Send teach_safe once the object is down.
  park_after_pick: true

  # string, default: /arm/enable_flag. std_msgs/Bool the skill latches to
  # release the arm (false = DisableArm) for home(disable=true). It is a deploy
  # constant rather than an atlas-resolved endpoint: the driver's own
  # /arm/enable_srv speaks piper_msgs, which only exists in the arm package's
  # colcon overlay.
  arm_enable_topic: /arm/enable_flag
