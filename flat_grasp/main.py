# SPDX-License-Identifier: Apache-2.0
"""flat_grasp — flat-palm enveloping grasp skill for Piper + LinkerHand O6.

Owns `robonix/skill/pick/*`. Exposes two MCP tools:

    grab(object_name)  grasp the named object and KEEP HOLDING it
    pick(object_name)  grasp it, then put it back down

Pipeline (every endpoint resolved through atlas — no hardcoded topic names,
URLs or ports):

    detect_object    mcp   service/perception/object_detect/detect_object
    grasp_request    grpc  service/perception/grasp_pose/grasp_request
    execute_grasp    grpc  service/manipulation/execute_grasp
    teach_safe       grpc  service/manipulation/teach_safe
    hand_move        grpc  primitive/hand/move_joint
    hand_state       grpc  primitive/hand/get_state

Why this skill exists at all: each of those covers one layer, and the grasp
SEQUENCE — approach above, descend, close the FINGERS, lift, and (for pick)
reverse it — is not any single service's job. `execute_grasp` moves the arm
and nothing else; the LinkerHand is a separate primitive on a separate
transport, so somebody has to own the ordering. That is this file.

Adapted from /home/czn/wfw/robot-agilex-piper/packages/
skill-pick-vertical-grasp-rbnx (same arm, but a vertical gripper grasp). The
flat-palm geometry itself is NOT here — it lives in grasp_pose's flat mode
(pixel -> arm pose) and roboarm_ik's solve_flat (pose -> joints).
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import threading
import time
from typing import Any, Optional

import numpy as np
from robonix_api import ATLAS, Err, Ok, Skill

logging.basicConfig(
    level=os.environ.get("FLAT_GRASP_LOG_LEVEL", "INFO"),
    format="[flat_grasp] %(message)s",
)
log = logging.getLogger("flat_grasp")

flat_grasp = Skill(id="flat_grasp", namespace="robonix/skill/pick")

# ── upstream contracts ───────────────────────────────────────────────────
REQUIRED_INPUTS = {
    "detect_object": ("robonix/service/perception/object_detect/detect_object", "mcp"),
    "grasp_request": ("robonix/service/perception/grasp_pose/grasp_request", "grpc"),
    "execute_grasp": ("robonix/service/manipulation/execute_grasp", "grpc"),
    # The hand speaks the GLOBAL robonix hand contracts, which define commands as
    # rpc — not the ros2 topic pair this skill used to publish on. See the hand
    # primitive's package README for why `state_joint` (a topic_out contract) is
    # not served: its IDL only exists as a codegen search root, so serving it
    # would mean vendoring a fork of a global message definition.
    "hand_move": ("robonix/primitive/hand/move_joint", "grpc"),
    "hand_state": ("robonix/primitive/hand/get_state", "grpc"),
}
# teach_safe and reset are optional: a missing one only costs us the ability to
# park the arm (teach_safe) or to send it to all-zeros (reset). Both live on
# roboarm_ik, so they resolve to the same endpoint.
OPTIONAL_INPUTS = {
    "teach_safe": ("robonix/service/manipulation/teach_safe", "grpc"),
    "reset": ("robonix/service/manipulation/reset", "grpc"),
}

# ── defaults (overridable from the manifest's skill[].config) ────────────
# Axis names come from the hand primitive's `info` contract and are fixed by the
# global hand contract set — not ours to choose.
DEFAULT_HAND_JOINTS = [
    "thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
    "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch",
]
# Poses are in CONTRACT units: normalized [0,1] with 0 = open and 1 = closed.
# These are roboarm's tuned O6 RIGHT-hand poses (arm/linker_hand.py
# DEFAULT_OPEN_PALM_POSES / DEFAULT_FIST_POSES), converted on the way in.
#
# The conversion is `contract = 1 - sdk/255` — note the INVERSION. The firmware
# uses 255 = fully open, while the contract uses 1 = closed / 0 = open. To
# convert a value you read out of roboarm's config: 1 - v/255.
#   open  sdk [255,  70, 255, 255, 255, 255]
#       ->    [0.000, 0.725, 0.000, 0.000, 0.000, 0.000]
#   fist  sdk [102,  18,   0,   0,   0,   0]
#       ->    [0.600, 0.929, 1.000, 1.000, 1.000, 1.000]
DEFAULT_HAND_OPEN = [0.0, 0.725, 0.0, 0.0, 0.0, 0.0]
DEFAULT_HAND_CLOSE = [0.6, 0.929, 1.0, 1.0, 1.0, 1.0]

_state_lock = threading.Lock()
_initialized = False
_activated = False
_cfg: dict[str, Any] = {}
_endpoints: dict[str, str] = {}
# Where the last successful grasp happened. `put_down` releases the object
# here, because this deploy has no taught place pose — the grasp point is the
# only location it can return to with confidence.
_last_grasp: Optional[dict[str, Any]] = None

# ROS side (owned here only for the arm's enable flag — the hand goes over gRPC)
_ros_node = None
_enable_pub = None
_ros_thread: Optional[threading.Thread] = None
_ros_stop = threading.Event()

# gRPC client caches
_grpc_lock = threading.Lock()
_grpc_channels: dict[str, Any] = {}
_grpc_stubs: dict[str, Any] = {}


# ── config helpers ───────────────────────────────────────────────────────
def _num(key: str, default: float) -> float:
    try:
        return float(_cfg.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def _float_list(key: str, default: list[float]) -> list[float]:
    v = _cfg.get(key)
    if not v:
        return list(default)
    return [float(x) for x in v]


def hand_open_pose() -> list[float]:
    return _float_list("hand_open_pose", DEFAULT_HAND_OPEN)


def hand_close_pose() -> list[float]:
    return _float_list("hand_close_pose", DEFAULT_HAND_CLOSE)


def hand_joint_names() -> list[str]:
    v = _cfg.get("hand_joint_names")
    return [str(x) for x in v] if v else list(DEFAULT_HAND_JOINTS)


# ── atlas resolution ─────────────────────────────────────────────────────
def _resolve_inputs(deadline_s: float = 60.0) -> dict[str, str]:
    """Block until atlas resolves every REQUIRED_INPUTS endpoint."""
    resolved: dict[str, str] = {}
    deadline = time.time() + deadline_s
    while time.time() < deadline:
        for key, (cid, transport) in REQUIRED_INPUTS.items():
            if key in resolved:
                continue
            try:
                cap = ATLAS.find_unique_capability(
                    contract_id=cid, transport=transport)
                ch = flat_grasp.connect_capability(cap, cid, transport)
            except Exception:  # noqa: BLE001
                continue
            ep = ch.endpoint
            try:
                ch.close()
            except Exception:  # noqa: BLE001
                pass
            if ep:
                resolved[key] = ep
                log.info("resolved %s [%s] -> %s", cid, transport, ep)
        if len(resolved) == len(REQUIRED_INPUTS):
            break
        time.sleep(2.0)

    missing = [k for k in REQUIRED_INPUTS if k not in resolved]
    if missing:
        raise RuntimeError(
            f"atlas could not resolve {missing} within {deadline_s:.0f}s "
            f"(resolved: {sorted(resolved)})"
        )
    for key, (cid, transport) in OPTIONAL_INPUTS.items():
        try:
            cap = ATLAS.find_unique_capability(
                contract_id=cid, transport=transport)
            ch = flat_grasp.connect_capability(cap, cid, transport)
            if ch.endpoint:
                resolved[key] = ch.endpoint
                log.info("resolved %s (optional) -> %s", cid, ch.endpoint)
            ch.close()
        except Exception:  # noqa: BLE001
            log.info("optional %s not available", cid)
    return resolved


# ── MCP client (detect_object) ───────────────────────────────────────────
# Dedicated background asyncio loop. robonix_api's MCP server (fastmcp
# streamable_http) invokes our handler from a thread that already has a running
# event loop, so asyncio.run() would raise "cannot be called from a running
# event loop". Standard fix: own a separate loop on a daemon thread and hop
# onto it with run_coroutine_threadsafe.
_bg_loop: Optional[asyncio.AbstractEventLoop] = None
_bg_loop_lock = threading.Lock()
_mcp_clients: dict[str, Any] = {}


def _ensure_bg_loop() -> asyncio.AbstractEventLoop:
    global _bg_loop
    with _bg_loop_lock:
        if _bg_loop is None or _bg_loop.is_closed():
            loop = asyncio.new_event_loop()
            threading.Thread(
                target=loop.run_forever, daemon=True, name="flat-grasp-mcp",
            ).start()
            _bg_loop = loop
        return _bg_loop


async def _mcp_call(url: str, tool: str, args: dict) -> dict:
    from fastmcp import Client

    client = _mcp_clients.get(url)
    if client is None:
        client = Client(url)
        _mcp_clients[url] = client
    async with client as c:
        result = await c.call_tool(tool, args)
        if not result.content:
            return {}
        txt = result.content[0].text
        try:
            return json.loads(txt)
        except Exception:  # noqa: BLE001
            return {"raw": txt}


def _mcp_call_sync(url: str, tool: str, args: dict, timeout_s: float = 30.0) -> dict:
    fut = asyncio.run_coroutine_threadsafe(_mcp_call(url, tool, args),
                                           _ensure_bg_loop())
    try:
        return fut.result(timeout=timeout_s)
    except Exception as e:  # noqa: BLE001
        log.warning("mcp call %s failed: %s", tool, e)
        return {"_error": str(e)}


# ── gRPC clients ─────────────────────────────────────────────────────────
def _channel_for(endpoint: str):
    with _grpc_lock:
        ch = _grpc_channels.get(endpoint)
        if ch is None:
            import grpc
            ch = grpc.insecure_channel(
                endpoint, options=[("grpc.enable_http_proxy", 0)])
            _grpc_channels[endpoint] = ch
        return ch


def _stub(key: str, endpoint: str, factory):
    """Cached gRPC stub for an atlas-resolved endpoint.

    NOTE the locking: `_channel_for` takes `_grpc_lock` itself, so this
    function must NOT call it while holding that lock — `_grpc_lock` is a plain
    (non-reentrant) Lock, and nesting it deadlocks the calling thread silently.
    That failure mode is nasty precisely because it looks like a hung network
    call: no exception, the deadline parameter never fires, and the provider
    never even sees the request. Build the channel outside the lock.
    """
    with _grpc_lock:
        stub = _grpc_stubs.get(key)
    if stub is not None:
        return stub

    channel = _channel_for(endpoint)
    with _grpc_lock:
        stub = _grpc_stubs.get(key)
        if stub is None:
            stub = factory(channel)
            _grpc_stubs[key] = stub
        return stub


def _grasp_request(bbox_2d: list[float], object_name: str) -> dict:
    import grasp_pb2
    import robonix_contracts_pb2_grpc as cg

    stub = _stub("grasp_request", _endpoints["grasp_request"],
                 cg.RobonixServicePerceptionGraspPoseGraspRequestStub)
    r = stub.GraspRequest(
        grasp_pb2.GraspRequest_Request(
            object_name=object_name, bbox_2d=list(bbox_2d)),
        timeout=_num("grasp_request_timeout_s", 10.0))
    p = r.grasp_pose.pose.position
    o = r.grasp_pose.pose.orientation
    return {
        "success": bool(r.success),
        "message": str(r.message),
        "frame_id": str(r.grasp_pose.header.frame_id),
        "xyz": [float(p.x), float(p.y), float(p.z)],
        "quat": [float(o.x), float(o.y), float(o.z), float(o.w)],
        "object_width": float(r.gripper_width),
    }


def _execute_grasp(xyz, quat, timeout_s: float) -> dict:
    import geometry_msgs_pb2
    import manipulation_pb2
    import robonix_contracts_pb2_grpc as cg
    import std_msgs_pb2

    stub = _stub("execute_grasp", _endpoints["execute_grasp"],
                 cg.RobonixServiceManipulationExecuteGraspStub)
    req = manipulation_pb2.ExecuteGrasp_Request(
        target_pose=geometry_msgs_pb2.PoseStamped(
            header=std_msgs_pb2.Header(frame_id="arm/base_link"),
            pose=geometry_msgs_pb2.Pose(
                position=geometry_msgs_pb2.Point(
                    x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
                orientation=geometry_msgs_pb2.Quaternion(
                    x=float(quat[0]), y=float(quat[1]),
                    z=float(quat[2]), w=float(quat[3])),
            ),
        ),
        # This robot has no gripper; the hand is driven separately. (The Piper
        # driver still logs "gripper closed" with gripper_exist: false — the
        # entry is published but nothing acts on it.)
        gripper_width=0.0,
        timeout_s=float(timeout_s),
    )
    r = stub.ExecuteGrasp(req, timeout=timeout_s + 15.0)
    return {"success": bool(r.success), "message": str(r.message),
            "elapsed_s": float(r.elapsed_s)}


def _teach_safe() -> dict:
    import manipulation_pb2
    import robonix_contracts_pb2_grpc as cg

    if "teach_safe" not in _endpoints:
        return {"success": False, "message": "teach_safe not resolved"}
    stub = _stub("teach_safe", _endpoints["teach_safe"],
                 cg.RobonixServiceManipulationTeachSafeStub)
    r = stub.TeachSafe(
        manipulation_pb2.TeachSafe_Request(hold_gripper=False),
        timeout=_num("teach_safe_timeout_s", 25.0))
    return {"success": bool(r.success), "message": str(r.message),
            "elapsed_s": float(r.elapsed_s)}


def _reset_zero() -> dict:
    """Send the arm to all six joints at zero (roboarm_ik's init_joints_deg)."""
    import manipulation_pb2
    import robonix_contracts_pb2_grpc as cg

    if "reset" not in _endpoints:
        return {"success": False, "message": "reset not resolved"}
    stub = _stub("reset", _endpoints["reset"],
                 cg.RobonixServiceManipulationResetStub)
    r = stub.Reset(manipulation_pb2.Reset_Request(),
                   timeout=_num("teach_safe_timeout_s", 25.0))
    return {"success": bool(r.success), "message": str(r.message),
            "elapsed_s": float(r.elapsed_s)}


# ── ROS: the hand ────────────────────────────────────────────────────────
def _ros_thread_main() -> None:
    """Own a minimal rclpy node for the arm's enable flag.

    The HAND is no longer driven from here: it speaks the global hand contracts
    over gRPC (see _hand_move / _hand_read). All that is left on ROS in
    this skill is the Piper driver's enable flag, which is a bare
    std_msgs/Bool topic rather than a declared capability.
    """
    global _ros_node, _enable_pub
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    if not rclpy.ok():
        rclpy.init()
    node = Node("flat_grasp_ros")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    # The Piper driver reads `false` here as DisableArm(7). It is a stock
    # std_msgs/Bool, which is why this route is used instead of the driver's
    # /arm/enable_srv — that service needs piper_msgs, a custom ROS package that
    # only exists inside the arm primitive's colcon overlay.
    _enable_pub = node.create_publisher(
        Bool, str(_cfg.get("arm_enable_topic", "/arm/enable_flag")), qos)
    _ros_node = node
    log.info("arm enable-flag publisher up: %s",
             _cfg.get("arm_enable_topic", "/arm/enable_flag"))

    while not _ros_stop.is_set():
        try:
            rclpy.spin_once(node, timeout_sec=0.2)
        except Exception:  # noqa: BLE001
            if _ros_stop.is_set():
                break
            log.exception("ros spin error")
    try:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:  # noqa: BLE001
        pass


def _hand_move(pose: list[float]) -> None:
    """Drive all six axes in one move_joint call (contract units, 0=open/1=closed).

    One call rather than six: the primitive writes the whole hand in a single
    Modbus frame, so the fingers never pass through an unintended intermediate
    shape. Raises on rejection — the primitive rejects (never clamps) unknown
    names and out-of-range values, and swallowing that here would leave the hand
    somewhere the caller did not ask for.

    Stub comes from robonix_contracts_pb2_grpc, not hand_pb2_grpc: robonix puts
    every service stub in that one combined module (hand_pb2_grpc is generated
    but carries only the version header).
    """
    import hand_pb2
    import robonix_contracts_pb2_grpc as contracts_grpc

    names = hand_joint_names()
    if len(pose) != len(names):
        raise ValueError(
            f"hand pose has {len(pose)} values but {len(names)} axis names")
    stub = _stub("hand_move", _endpoints["hand_move"],
                 contracts_grpc.RobonixPrimitiveHandMoveJointStub)
    resp = stub.MoveJoint(
        hand_pb2.MoveJoint_Request(
            targets=[hand_pb2.JointValue(name=n, value=float(v))
                     for n, v in zip(names, pose)]),
        timeout=_num("hand_move_timeout_s", 5.0))
    if not resp.ok:
        raise RuntimeError(f"move_joint rejected: {resp.message}")
    log.info("hand -> %s", [round(v, 3) for v in pose])


def _hand_read() -> Optional[list[float]]:
    """Current axis positions in contract units, or None if the read failed."""
    import hand_pb2
    import robonix_contracts_pb2_grpc as contracts_grpc

    if "hand_state" not in _endpoints:
        return None
    try:
        stub = _stub("hand_state", _endpoints["hand_state"],
                     contracts_grpc.RobonixPrimitiveHandGetStateStub)
        resp = stub.GetState(hand_pb2.GetState_Request(),
                             timeout=_num("hand_state_timeout_s", 5.0))
    except Exception as e:  # noqa: BLE001
        log.warning("get_state failed: %s", e)
        return None
    if not resp.ok:
        log.warning("get_state rejected: %s", resp.message)
        return None
    order = {n: i for i, n in enumerate(hand_joint_names())}
    out = [0.0] * len(order)
    for jv in resp.joints:
        if jv.name in order:
            out[order[jv.name]] = float(jv.value)
    return out


def _set_arm_enable(enable: bool, repeats: int = 5) -> None:
    """Drive the Piper driver's enable flag. False = DisableArm (失能, not 断电).

    The drivers stay powered and the arm re-enables on demand — this is the
    same operation as roboarm's Arm.disable_torque(), reached over ROS instead
    of through the SDK.
    """
    from std_msgs.msg import Bool

    msg = Bool()
    msg.data = bool(enable)
    # Repeated: a single dropped frame would leave the arm enabled (or, worse,
    # leave a disable command unheeded when the operator asked for one).
    for _ in range(repeats):
        if _enable_pub is not None:
            _enable_pub.publish(msg)
        time.sleep(_num("hand_publish_gap_s", 0.15))
    log.info("arm enable_flag -> %s", enable)


def _resolve_place_xy(object_name: str, grasp_xyz: list[float]) -> tuple[list[float], str]:
    """Where to put the object down, as ([x, y, z], human-readable source).

    Mirrors roboarm's `place_pos` config (config.yaml.example + the matching
    code in classification/catch_with_linker_hand_flat.py), including its
    default: **an object with no entry goes back where it came from.**

    Config shape, keyed by the detector's class name:

        place_pos:
          tomato:
            pos: [0.30, -0.10]      # literal metres in arm/base_link
          potato:
            pos: ["x", "-y"]        # relative to the object's own position

    A `pos` element may be a number, or one of the symbolic refs "x" / "-x" /
    "y" / "-y", which substitute the object's CURRENT grasp coordinate (sign
    flipped for the negatives). That is roboarm's scheme: "put every potato at
    the object's x, mirrored across y" is one line rather than a taught pose.

    `pos` may be [x, y] or [x, y, z]. A missing z means "the same height we
    grasped at", which is the flat-grasp working height — placing anywhere else
    vertically would either drop the object or press it into the table.

    Keyword matching is NOT done here. roboarm needs it because its LLM can
    name an object the detector doesn't know; this deploy passes the detected
    class name straight through, so an exact key is the honest lookup. A
    misspelt key therefore falls back to the grasp point rather than silently
    matching something else.
    """
    table = _cfg.get("place_pos") or {}
    entry = table.get(object_name) or {}
    pos = entry.get("pos") if isinstance(entry, dict) else None
    if not pos or len(pos) not in (2, 3):
        return list(grasp_xyz), "grasp point (no place_pos entry)"

    def _one(ref) -> float:
        if isinstance(ref, (int, float)):
            return float(ref)
        key = str(ref).strip().lower()
        sign = -1.0 if key.startswith("-") else 1.0
        key = key.lstrip("-")
        if key == "x":
            return sign * float(grasp_xyz[0])
        if key == "y":
            return sign * float(grasp_xyz[1])
        if key == "z":
            return sign * float(grasp_xyz[2])
        raise ValueError(
            f"place_pos[{object_name!r}].pos element {ref!r} is neither a "
            f"number nor one of 'x' / '-x' / 'y' / '-y' / 'z' / '-z'")

    try:
        xy = [_one(pos[0]), _one(pos[1])]
        z = _one(pos[2]) if len(pos) == 3 else float(grasp_xyz[2])
    except ValueError as e:
        log.warning("bad place_pos entry, falling back to the grasp point: %s", e)
        return list(grasp_xyz), "grasp point (bad place_pos entry)"

    xyz = [xy[0], xy[1], z]
    thr = _num("place_distance_threshold_m", 0.0)
    if thr > 0 and math.hypot(xy[0] - grasp_xyz[0], xy[1] - grasp_xyz[1]) < thr:
        return list(grasp_xyz), "grasp point (already within place_distance_threshold_m)"
    return xyz, f"place_pos[{object_name!r}]"


def _place_quaternion(place_xy: list[float]) -> list[float]:
    """Orientation for the place point.

    Same rule the grasp uses, applied at the DESTINATION: the wrist faces the
    place point's own bearing, i.e. Rz(atan2(y, x) + phi0) @ R_taught. Reusing
    the grasp quaternion would leave the palm aimed at wherever the object came
    from, which for a place point on the other side of the base is 90 degrees
    away or more.
    """
    from scipy.spatial.transform import Rotation as R

    phi0 = _num("place_heading_offset_deg", 4.2)
    taught = _cfg.get("flat_euler_deg_zyx") or [-142.249, 81.015, 149.475]
    heading = math.atan2(place_xy[1], place_xy[0]) + math.radians(phi0)
    rot = (R.from_rotvec([0.0, 0.0, heading])
           * R.from_euler("zyx", [float(v) for v in taught], degrees=True))
    return [float(v) for v in rot.as_quat()]


def _verify_grasp(close_pose: list[float]) -> tuple[bool, str]:
    """Did the fingers actually stop against something?

    Robonix's own Pick.srv defines `success` as "the arm completed the grasp
    motion AND fresh gripper feedback confirmed that an object is held" — so
    finishing the motion is not enough. Without this check the skill happily
    reports success after closing on thin air, and the caller has no way to
    tell: the arm looks right, the sequence completed, and the only evidence is
    the axis positions.

    The signal is simple. Commanded closed is 1.0 on every finger axis; an EMPTY
    hand reaches all of it, while the O6 holding this tomato stops its four
    fingers around 0.42-0.51 (contract units). So: if every finger axis reached
    (within tolerance) the fully-closed command, nothing was in the way.

    Only the four finger axes are judged. The thumb's pitch and yaw are excluded
    because in the flat-palm posture they wrap around the object's side and
    their travel is not a reliable contact indicator.

    Verified on real hardware 2026-09-14 (recorded in firmware counts, converted
    here by 1 - sdk/255): tomato held -> fingers 148/145/145/129 -> contract
    0.42/0.43/0.43/0.49 (held); hand closed on air -> 0/0/0/0 -> 1.0/1.0/1.0/1.0
    (empty). The gap between those outcomes is ~0.5, so the default 0.12
    tolerance is nowhere near a judgement call.
    """
    achieved = _hand_read()
    if achieved is None:
        # No readback is not evidence of failure — the hand primitive might be
        # restarting. Say so, and let the grasp stand rather than failing a
        # possibly-good pick.
        log.warning("no hand readback; cannot verify the grasp — assuming held")
        return True, "unverified (no get_state readback)"

    finger_idx = list(range(2, min(6, len(achieved))))
    if not finger_idx:
        return True, "unverified (no finger axes in readback)"
    tol = _num("grasp_verify_tolerance", 0.12)   # contract units, [0,1]
    worst = max(abs(achieved[i] - float(close_pose[i])) for i in finger_idx)
    detail = "fingers at %s, commanded %s, worst gap %.3f" % (
        [round(achieved[i], 3) for i in finger_idx],
        [round(float(close_pose[i]), 3) for i in finger_idx], worst)
    if worst <= tol:
        return False, "empty hand — %s" % detail
    return True, "held — %s" % detail


# ── the grasp sequences ──────────────────────────────────────────────────
def _detect(object_name: str) -> dict:
    """Run the detector.

    The DetectObject IDL returns only bbox_2d / object_center_3d / confidence /
    success / message — there is no class-name field, so the matched name is
    whatever the caller asked for. bbox_2d is the YOLO server's 5-element form
    (axis-aligned box + OBB rotation in degrees); grasp_pose tolerates both.
    """
    log.info("detect_object(%r) -> %s", object_name, _endpoints["detect_object"])
    resp = _mcp_call_sync(
        _endpoints["detect_object"], "detect_object",
        {"object_name": object_name, "backend": ""},
        timeout_s=_num("detect_timeout_s", 30.0))
    log.info("detect raw response: %s", str(resp)[:300])
    if resp.get("_error"):
        return {"success": False, "message": f"detect call failed: {resp['_error']}"}
    if not resp.get("success"):
        return {"success": False,
                "message": f"detection_failed: {resp.get('message', resp)}"}
    bbox = list(resp.get("bbox_2d") or [])
    if len(bbox) not in (4, 5):
        return {"success": False,
                "message": f"detector returned bbox_2d={bbox!r}, expected 4 or 5 values"}
    return {"success": True, "bbox_2d": bbox,
            "object_name": object_name,
            "confidence": float(resp.get("confidence", 0.0) or 0.0)}


def _grasp_and_hold(object_name: str) -> dict:
    """detect -> pose -> descend -> close fingers -> lift. Leaves it held."""
    # Declared at the top: Python rejects a `global` that follows any use of the
    # name in the same function, and this one is assigned deep inside the
    # success path — reading it anywhere above would turn a valid function into
    # a SyntaxError at import.
    global _last_grasp

    motion_to = _num("motion_timeout_s", 25.0)
    settle = _num("settle_s", 1.5)

    det = _detect(object_name)
    if not det["success"]:
        return det
    log.info("detect ok: bbox_2d=%s conf=%.3f",
             [round(v, 1) for v in det["bbox_2d"]], det["confidence"])

    gp = _grasp_request(det["bbox_2d"], object_name)
    if not gp["success"]:
        return {"success": False, "message": f"grasp_pose_failed: {gp['message']}"}
    x, y, z = gp["xyz"]
    quat = gp["quat"]
    log.info("grasp target xyz=(%.4f, %.4f, %.4f) width=%.1f mm frame=%s",
             x, y, z, gp["object_width"] * 1000.0, gp["frame_id"])

    # Open BEFORE approaching: closing the palm on the way in would knock the
    # object over instead of enveloping it.
    _hand_move(hand_open_pose())
    time.sleep(settle)

    # Approach height is not a free parameter. Holding the palm parallel to
    # the table costs wrist range, and that cost grows with height: measured
    # on this arm, an object at x=0.223 has IK cost 3.5 at z=0.11, 0.2 at
    # z=0.16, 8.0 at z=0.21 and 17.8 at z=0.24 (success threshold 4.0). So a
    # tall approach offset can be UNREACHABLE for near-in objects even though
    # the grasp pose itself is fine.
    #
    # Rather than pick one offset that happens to work from here, walk down
    # from the configured one until the IK converges. Failing an IK leaves the
    # arm untouched (execute_grasp returns before publishing anything), so
    # retrying lower is safe — it just costs a round trip.
    full = _num("safe_z_offset_m", 0.05)
    offsets = [full, full * 0.75, full * 0.5, full * 0.25]
    attempt_err: Optional[str] = None
    for off in offsets:
        r = _execute_grasp([x, y, z + off], quat, motion_to)
        if r["success"]:
            safe_z = z + off
            if off != full:
                log.info("approach at +%.3f m (configured +%.3f) — the "
                         "configured offset was not reachable here",
                         off, full)
            break
        attempt_err = r["message"]
        log.info("approach at +%.3f m failed: %s", off, r["message"])
    else:
        return {"success": False,
                "message": f"execute_grasp_failed (approach): {attempt_err}"}

    r = _execute_grasp([x, y, z], quat, motion_to)
    if not r["success"]:
        return {"success": False,
                "message": f"execute_grasp_failed (descend): {r['message']}"}
    time.sleep(settle)

    _hand_move(hand_close_pose())
    time.sleep(_num("close_settle_s", 2.0))

    held, verdict = _verify_grasp(hand_close_pose())
    log.info("grasp verification: %s", verdict)

    # Lift either way: if the hand is empty we still want it clear of the table
    # before anyone retries, and leaving it resting on the surface is a worse
    # state to hand back.
    r = _execute_grasp([x, y, safe_z], quat, motion_to)
    if not r["success"]:
        return {"success": False,
                "message": f"execute_grasp_failed (lift): {r['message']}"}

    if not held:
        # Open again so the next attempt starts from a known hand state.
        _hand_move(hand_open_pose())
        return {"success": False,
                "message": f"grasp_verify_failed: {verdict}"}

    with _state_lock:
        _last_grasp = {"xyz": [x, y, z], "quat": list(quat), "safe_z": safe_z,
                       "settle": settle, "motion_to": motion_to,
                       "object_name": det["object_name"]}

    return {"success": True, "xyz": [x, y, z], "quat": quat, "safe_z": safe_z,
            "object_width": gp["object_width"], "score": det["confidence"],
            "object_name": det["object_name"],
            "message": f"grasped {det['object_name']!r} ({verdict})"}


def _release_at(xyz, quat, safe_z: float, settle: float,
                motion_to: float) -> dict:
    r = _execute_grasp(xyz, quat, motion_to)
    if not r["success"]:
        return {"success": False,
                "message": f"execute_grasp_failed (to place): {r['message']}"}
    time.sleep(settle)
    _hand_move(hand_open_pose())
    time.sleep(_num("release_settle_s", 1.5))
    r = _execute_grasp([xyz[0], xyz[1], safe_z], quat, motion_to)
    if not r["success"]:
        return {"success": False,
                "message": f"execute_grasp_failed (retreat): {r['message']}"}
    return {"success": True}


# ── MCP tool request/response types ──────────────────────────────────────
# These come from codegen's robonix_mcp_types (generated with `rbnx codegen
# --mcp`), NOT from pick_pb2. robonix_api's @.mcp decorator calls
# `input_cls.json_schema()` when it builds the tool shim, and protobuf
# messages have no such method — typing a handler against pick_pb2 dies at
# import with a bare `AttributeError: json_schema`, before the package can
# even register with atlas.
import builtin_interfaces_mcp  # noqa: E402
import geometry_msgs_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from pick_mcp import (  # noqa: E402
    Home_Request,
    Home_Response,
    Pick_Request,
    Pick_Response,
    PutDown_Request,
    PutDown_Response,
)


def _pose_stamped(xyz, quat) -> "geometry_msgs_mcp.PoseStamped":
    return geometry_msgs_mcp.PoseStamped(
        header=std_msgs_mcp.Header(
            stamp=builtin_interfaces_mcp.Time(sec=int(time.time()), nanosec=0),
            frame_id="arm/base_link",
        ),
        pose=geometry_msgs_mcp.Pose(
            position=geometry_msgs_mcp.Point(
                x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2])),
            orientation=geometry_msgs_mcp.Quaternion(
                x=float(quat[0]), y=float(quat[1]),
                z=float(quat[2]), w=float(quat[3])),
        ),
    )


@flat_grasp.mcp("robonix/skill/pick/grab")
def grab(req: Pick_Request) -> Pick_Response:
    """Grasp the named object and keep holding it."""
    t0 = time.monotonic()
    name = str(req.object_name or "").strip()
    if not name:
        return Pick_Response(success=False, message="object_name is required",
                             elapsed_s=0.0)
    try:
        _ensure_activated()
        res = _grasp_and_hold(name)
    except Exception as e:  # noqa: BLE001
        log.exception("grab failed")
        return Pick_Response(success=False, message=f"grab exception: {e}",
                             elapsed_s=float(time.monotonic() - t0))
    resp = Pick_Response(
        success=bool(res["success"]),
        message=str(res.get("message", "")),
        elapsed_s=float(time.monotonic() - t0),
    )
    if res["success"]:
        resp.grasp_pose = _pose_stamped(res["xyz"], res["quat"])
        resp.gripper_width = float(res["object_width"])
        resp.score = float(res["score"])
    return resp


@flat_grasp.mcp("robonix/skill/pick/pick")
def pick(req: Pick_Request) -> Pick_Response:
    """Grasp the named object and put it back down.

    There is no taught place pose in this deploy (roboarm_ik's
    put_down_joints_deg is unset), so `pick` returns the object to the spot it
    was taken from — the grasp XY at grasp height. That needs no taught target
    and cannot drop the object somewhere unintended.
    """
    t0 = time.monotonic()
    name = str(req.object_name or "").strip()
    if not name:
        return Pick_Response(success=False, message="object_name is required",
                             elapsed_s=0.0)

    def _fail(msg: str) -> Pick_Response:
        return Pick_Response(success=False, message=msg,
                             elapsed_s=float(time.monotonic() - t0))

    try:
        _ensure_activated()
        motion_to = _num("motion_timeout_s", 25.0)
        settle = _num("settle_s", 1.5)

        res = _grasp_and_hold(name)
        if not res["success"]:
            return _fail(res.get("message", "grasp failed"))

        place_xy, place_src = _resolve_place_xy(res["object_name"], res["xyz"])
        place_quat = _place_quaternion(place_xy)
        log.info("place target = (%.3f, %.3f) z=%.3f  [%s]",
                 place_xy[0], place_xy[1], res["xyz"][2], place_src)

        released = _release_at(place_xy, place_quat, res["safe_z"],
                               settle, motion_to)
        if not released["success"]:
            # The object is still in the hand — say so plainly rather than
            # returning a generic failure a caller might retry blindly.
            return _fail(f"{released['message']}; object still held")

        if _cfg.get("park_after_pick", True):
            ts = _teach_safe()
            if not ts["success"]:
                log.warning("teach_safe after pick failed: %s", ts["message"])

        resp = Pick_Response(
            success=True,
            message=(f"picked {name!r} and placed it at "
                     f"({place_xy[0]:.3f}, {place_xy[1]:.3f}) — {place_src}"),
            elapsed_s=float(time.monotonic() - t0),
        )
        resp.grasp_pose = _pose_stamped(res["xyz"], res["quat"])
        resp.gripper_width = float(res["object_width"])
        resp.score = float(res["score"])
        return resp
    except Exception as e:  # noqa: BLE001
        log.exception("pick failed")
        return _fail(f"pick exception: {e}")


@flat_grasp.mcp("robonix/skill/pick/put_down")
def put_down(req: PutDown_Request) -> PutDown_Response:
    """Release a HELD object at the point it was grasped from.

    The counterpart to `grab`. The executor needs this because `pick` is NOT a
    release: `pick` re-runs detection and grasping, which opens the fingers on
    approach and drops whatever was held before picking it up again. Asked to
    "放下", the planner reached for `pick` and said so itself (2026-09-14).
    """
    t0 = time.monotonic()

    def _fail(msg: str) -> PutDown_Response:
        return PutDown_Response(success=False, message=msg,
                                elapsed_s=float(time.monotonic() - t0))

    global _last_grasp

    try:
        _ensure_activated()
    except Exception as e:  # noqa: BLE001
        return _fail(f"activation failed: {e}")

    with _state_lock:
        last = dict(_last_grasp) if _last_grasp else None
    if last is None:
        return _fail("nothing to put down — no grasp has succeeded since this "
                     "skill started, so the release point is unknown")

    held, verdict = _verify_grasp(hand_close_pose())
    log.info("put_down precheck: %s", verdict)
    if not held:
        return _fail(f"nothing is held — {verdict}")

    place_xy, place_src = _resolve_place_xy(
        str(last.get("object_name", "")), last["xyz"])
    place_quat = _place_quaternion(place_xy)
    log.info("place target = (%.3f, %.3f)  [%s]",
             place_xy[0], place_xy[1], place_src)

    released = _release_at(place_xy, place_quat, last["safe_z"],
                           float(last["settle"]), float(last["motion_to"]))
    if not released["success"]:
        return _fail(f"{released['message']}; object may still be held")

    if _cfg.get("park_after_pick", True):
        ts = _teach_safe()
        if not ts["success"]:
            log.warning("teach_safe after put_down failed: %s", ts["message"])

    with _state_lock:
        _last_grasp = None
    return PutDown_Response(
        success=True,
        message=(f"released at ({place_xy[0]:.3f}, {place_xy[1]:.3f}) "
                 f"— {place_src}"),
        elapsed_s=float(time.monotonic() - t0))


@flat_grasp.mcp("robonix/skill/pick/home")
def home(req: Home_Request) -> Home_Response:
    """Park the arm, optionally de-energising it.

    Mirrors roboarm's arm/disable_arm.py (Arm.disconnect_arm):
        reset -> move_to_home(safe_pos=True) -> disable_torque
    The order is the point. Disabling while the arm is stretched out lets it
    sag under its own weight — the resting pose has to come first.
    """
    t0 = time.monotonic()

    def _fail(msg: str) -> Home_Response:
        return Home_Response(success=False, message=msg,
                             elapsed_s=float(time.monotonic() - t0))

    try:
        _ensure_activated()
    except Exception as e:  # noqa: BLE001
        return _fail(f"activation failed: {e}")

    target = str(getattr(req, "target", "") or "safe").strip().lower()
    disable = bool(getattr(req, "disable", False))
    if target in ("", "safe", "teach_safe", "middle"):
        move = _teach_safe()
        what = "teach-safe"
    elif target in ("zero", "zeros", "init", "home"):
        move = _reset_zero()
        what = "zero"
    else:
        return _fail(f"unknown target {target!r} (expected 'safe' or 'zero')")

    if not move["success"]:
        # Do NOT disable on a failed move: the arm is somewhere unintended and
        # going limp there is the one outcome worse than not parking.
        return _fail(f"move to {what} failed: {move['message']}")

    msg = f"moved to {what} ({move['elapsed_s']:.1f}s)"
    if disable:
        _set_arm_enable(False)
        msg += "; arm de-energised (standby, not powered off)"
    return Home_Response(success=True, message=msg,
                         elapsed_s=float(time.monotonic() - t0))


# ── lifecycle ────────────────────────────────────────────────────────────
@flat_grasp.on_init
def init(cfg):
    """Driver(CMD_INIT): validate config. No atlas, no ROS, no gRPC yet."""
    global _initialized, _cfg
    with _state_lock:
        if _initialized:
            return Ok()
    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as e:
            return Err(f"bad config_json: {e}")

    names = cfg.get("hand_joint_names") or DEFAULT_HAND_JOINTS
    open_pose = cfg.get("hand_open_pose") or DEFAULT_HAND_OPEN
    close_pose = cfg.get("hand_close_pose") or DEFAULT_HAND_CLOSE
    for label, pose in (("hand_open_pose", open_pose),
                        ("hand_close_pose", close_pose)):
        if len(pose) != len(names):
            return Err(
                f"{label} has {len(pose)} values but hand_joint_names has "
                f"{len(names)}")
        if any(not (0 <= int(v) <= 255) for v in pose):
            return Err(f"{label} values must be 0~255, got {pose}")

    _cfg = dict(cfg)
    with _state_lock:
        _initialized = True
    log.info("CMD_INIT ok")
    return Ok()


def _do_activate() -> None:
    """Resolve upstreams + bring up the hand publisher. Idempotent."""
    global _activated, _endpoints, _ros_thread
    with _state_lock:
        if _activated:
            return
    _endpoints = _resolve_inputs(deadline_s=_num("resolve_timeout_s", 60.0))

    _ros_stop.clear()
    _ros_thread = threading.Thread(
        target=_ros_thread_main, name="flat-grasp-ros", daemon=True)
    _ros_thread.start()
    # Wait for the enable-flag publisher to exist before anything can use it.
    # (The hand no longer needs a ROS publisher — it goes over gRPC.)
    for _ in range(100):
        if _enable_pub is not None:
            break
        time.sleep(0.1)
    if _enable_pub is None:
        _ros_stop.set()
        raise RuntimeError("arm enable-flag publisher did not come up")

    with _state_lock:
        _activated = True


def _ensure_activated() -> None:
    """Lazy activation for direct MCP calls.

    The executor sends Driver(CMD_ACTIVATE) only when IT routes a call (i.e.
    through pilot). A caller that connects to this skill's MCP endpoint
    directly — a test harness, or another provider — arrives with the skill
    still INACTIVE and every handler would die on an empty `_endpoints` with a
    bare KeyError. Activating on first use costs nothing (it is idempotent) and
    makes the skill work no matter who calls it.
    """
    if not _activated:
        _do_activate()


@flat_grasp.on_activate
def activate():
    """First executor-routed MCP call: resolve upstreams + start ROS."""
    try:
        _do_activate()
    except Exception as e:  # noqa: BLE001
        return Err(f"activation failed: {e}")
    log.info("CMD_ACTIVATE ok (endpoints=%s)", sorted(_endpoints))
    return Ok()


@flat_grasp.on_deactivate
def deactivate():
    global _activated
    _activated = False
    log.info("CMD_DEACTIVATE ok")
    return Ok()


if __name__ == "__main__":
    flat_grasp.run()
