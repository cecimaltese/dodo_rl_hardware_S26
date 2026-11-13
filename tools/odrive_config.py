"""
odrive_config.py — read, back up and copy the ODrives' own configuration over CAN (no USB).

odrivetool >= 0.6.9 talks to the drives through the SocketCAN bus as well as USB: it finds
them by serial number and reads/writes any setting. This wraps that for the fleet:

    python3 tools/odrive_config.py list      # serial, node id, firmware, state of each drive
    python3 tools/odrive_config.py watch     # live encoder position per node: move a joint by
                                             # hand to see which drive is which (read-only)
    python3 tools/odrive_config.py pins --drive 7   # turn that joint: do its encoder pins
                                             # toggle, does the count move? (read-only)
    python3 tools/odrive_config.py backup    # every drive's config -> setup/odrive_config/
    python3 tools/odrive_config.py set-node --serial 336437783432 --node-id 3
                                             # dry run; --write sets it and saves (reboots)
    python3 tools/odrive_config.py clone --from 4 --to <serial> --node-id 8
                                             # dry run: what would change on the target

`set-node` changes only the node id. `clone` is for a replaced board whose settings are
wrong: it copies every setting of a working drive of the same kind, or of a backup file
(`--from setup/odrive_config/....json`). It keeps the target's own calibration results
(CALIBRATION_KEYS: phase R/L, motor direction, ...), then sets the node id and saves.
Both refuse a node id another drive has: two drives on one id both obey every command
sent to it. (2026-11-13: the new left-knee board came with node id 7, the right knee's.
Both answered as node 7, and node 3 was missing.)

odrivetool's discovery gives a drive it connects to the lowest FREE node id when its own id
is taken. That lasts until the next power-up, and it is how the duplicate above turned up
as node 0. Every command here discovers all drives first, so that can't land on a used id.
Node 63 is "unaddressed". Two drives booting with the same saved id ended up both at 63 on
2026-11-13 (the ODrive addressing rule: a drive that sees its id claimed by another serial
drops to 63). RAM only: they boot with their saved id again. Address drives by serial then.

Config only: the motors stay idle. Stop motor_service first (refused if a service answers).
Afterwards: `watch` (the right joint moves the right node), odrive_check.py on that joint
(its full calibration), then the bring-up.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import struct
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import can                                      # noqa: E402

import dodo_ipc as ipc                          # noqa: E402
import fleet_config                             # noqa: E402

CONFIG_DIR = REPO / "setup" / "odrive_config"
NODE_KEY = "axis0.config.can.node_id"
# The target's own results from ITS calibration, never another drive's. motor.direction:
# "Automatically set based on encoder direction after calibration. Do not modify this
# manually" (ODrive API docs); the commutation scale's sign goes with it.
CALIBRATION_KEYS = (
    "axis0.config.motor.phase_resistance", "axis0.config.motor.phase_resistance_valid",
    "axis0.config.motor.phase_inductance", "axis0.config.motor.phase_inductance_valid",
    "axis0.config.motor.direction", "axis0.commutation_mapper.config.scale",
)
SPOT_CHECK = (NODE_KEY, "axis0.config.motor.current_soft_max",
              "axis0.commutation_mapper.config.use_index_gpio", "can.config.protocol")
CMD_HEARTBEAT, CMD_ENCODER, CMD_REBOOT = 0x01, 0x09, 0x16
REBOOT_SAVE = 1                                 # Reboot action: save_configuration()
AXIS_STATE_IDLE = 1
DISCOVER_S = 3.0


# ── pure logic (tested) ───────────────────────────────────────────

def same(a, b) -> bool:
    """Config equality as the drive stores it: float32, NaN == NaN."""
    if isinstance(a, float) or isinstance(b, float):
        try:
            return struct.pack("<f", float(a)) == struct.pack("<f", float(b))
        except (TypeError, ValueError):
            return False
    return a == b


def plan_clone(source: dict, target: dict, node_id: int) -> dict:
    """What a clone writes on the target. Returns
    writes: {key: value} to write (node id last), only where the target differs;
    kept:   {key: (target value, source value)} calibration results left as they are;
    missing: source keys this target's firmware doesn't have (left out)."""
    writes, kept = {}, {}
    for k, v in source.items():
        if k == NODE_KEY or k not in target:
            continue
        if k in CALIBRATION_KEYS:
            if not same(v, target[k]):
                kept[k] = (target[k], v)
        elif not same(v, target[k]):
            writes[k] = v
    if target.get(NODE_KEY) != node_id:
        writes[NODE_KEY] = node_id      # last: it takes effect immediately
    missing = sorted(k for k in source if k not in target)
    return {"writes": writes, "kept": kept, "missing": missing}


def node_owner(drives: dict, node_id: int, except_serial: str) -> str | None:
    """The serial of another drive already on `node_id`, if any. drives: {serial: node}."""
    for sn, node in drives.items():
        if node == node_id and sn != except_serial:
            return sn
    return None


def joint_of(node_id: int) -> str:
    return next((c.name for c in fleet_config.OD_JOINTS if c.node_id == node_id),
                f"node{node_id}")


# ── odrivetool over CAN ───────────────────────────────────────────

class Fleet:
    """Every drive odrivetool finds on the bus, kept discovered while in use, so that a
    connection never gets an id that is in use (see the module docstring)."""

    def __init__(self, channel: str):
        self.channel = channel

    async def __aenter__(self):
        from odrive.device_manager import (DeviceManagerDelegate, FilterSpec,
                                           get_device_manager)
        from odrive.libodrive import DeviceType
        self.dm = get_device_manager()
        self.found = {}
        fleet = self

        class Collect(DeviceManagerDelegate):
            def on_found_device(self, dev):
                if dev.info.device_type == DeviceType.RUNTIME:
                    fleet.found[dev.info.serial_number] = dev

        self.sub = self.dm.subscribe(FilterSpec(interfaces=[f"can:{self.channel}"]), Collect())
        await asyncio.sleep(DISCOVER_S)
        return self

    async def __aexit__(self, *exc):
        self.dm.unsubscribe(self.sub)

    def nodes(self) -> dict:
        return {sn: dev.info.node_id for sn, dev in self.found.items()}

    def serial_at(self, node_id: int) -> str:
        sns = [sn for sn, n in self.nodes().items() if n == node_id]
        if len(sns) != 1:
            sys.exit(f"{len(sns)} drives at node {node_id}: {sns}; give the serial instead"
                     if sns else f"no drive at node {node_id}; found {self.nodes()} "
                     "(give the serial instead; 63 = unaddressed, see the docstring)")
        return sns[0]

    async def connect(self, serial: str):
        if serial not in self.found:
            sys.exit(f"drive {serial} not found; found {self.nodes()}")
        return await self.dm.use_connection(self.found[serial])

    async def read(self, dev, *names):
        out = {}
        for n in names:
            try:
                out[n] = await dev.read(dev.try_get_prop_info(n))
            except Exception as e:                       # noqa: BLE001
                out[n] = f"? ({e})"
        return out


async def backup_one(fleet: Fleet, serial: str) -> tuple[dict, dict]:
    from odrive.legacy_config import backup_config
    dev = await fleet.connect(serial)
    try:
        cfg = await backup_config(dev)
        info = await fleet.read(dev, "fw_version_major", "fw_version_minor",
                                "fw_version_revision", "hw_version_major",
                                "hw_version_minor", "axis0.current_state")
    finally:
        fleet.dm.release_connection(dev)
    return cfg, info


def fw(info: dict) -> str:
    return "{fw_version_major}.{fw_version_minor}.{fw_version_revision}".format(**info)


# ── plain CAN (python-can) ────────────────────────────────────────

def heartbeats(channel: str, secs: float) -> dict:
    """{node: (axis_error, state, procedure_result, frames/s)} heard on the bus."""
    bus = can.Bus(channel=channel, interface=fleet_config.BUSTYPE)
    last, n = {}, {}
    t_end = time.monotonic() + secs
    try:
        while time.monotonic() < t_end:
            m = bus.recv(0.05)
            if m is None or m.is_remote_frame or (m.arbitration_id & 0x1F) != CMD_HEARTBEAT:
                continue
            node = m.arbitration_id >> 5
            last[node] = struct.unpack_from("<IBB", bytes(m.data))
            n[node] = n.get(node, 0) + 1
    finally:
        bus.shutdown()
    return {node: (*last[node], n[node] / secs) for node in last}


def version(channel: str, node_id: int) -> str:
    """Get_Version (RTR 0x00): the firmware of the first drive that answers on node_id."""
    bus = can.Bus(channel=channel, interface=fleet_config.BUSTYPE)
    try:
        bus.send(can.Message(arbitration_id=node_id << 5, is_remote_frame=True, dlc=8,
                             is_extended_id=False))
        t_end = time.monotonic() + 0.3
        while time.monotonic() < t_end:
            m = bus.recv(0.05)
            if m is not None and m.arbitration_id == node_id << 5 and not m.is_remote_frame:
                return "{}.{}.{}".format(*m.data[4:7])
    finally:
        bus.shutdown()
    return "?"


def save_over_can(channel: str, node_id: int) -> None:
    bus = can.Bus(channel=channel, interface=fleet_config.BUSTYPE)
    try:
        bus.send(can.Message(arbitration_id=(node_id << 5) | CMD_REBOOT,
                             data=bytes([REBOOT_SAVE]), is_extended_id=False))
    finally:
        bus.shutdown()


# ── commands ──────────────────────────────────────────────────────

async def cmd_list(args):
    """Discovery only (no connection, so no id is ever reassigned) + plain-CAN heartbeats
    and Get_Version."""
    async with Fleet(args.channel) as fleet:
        drives = fleet.nodes()
    hb = heartbeats(args.channel, 1.0)
    for sn, node in sorted(drives.items(), key=lambda kv: kv[1]):
        h = hb.get(node)
        state = f"state {h[1]}, heartbeat {h[3]:.0f}/s" if h else "no heartbeat"
        print(f"  node {node:2d}  {joint_of(node):16s} serial {sn}  fw {version(args.channel, node)}"
              f"  {state}")
    nodes = list(drives.values())
    dup = sorted({n for n in nodes if nodes.count(n) > 1})
    if dup:
        print(f"  DUPLICATE node ids {dup}: two drives obey the same commands. Fix with clone.")
    for c in fleet_config.OD_JOINTS:
        if c.node_id not in nodes:
            print(f"  missing: node {c.node_id} ({c.name})")


def cmd_watch(args):
    """Encoder position (motor turns) per node, a line every 0.2 s; read-only."""
    bus = can.Bus(channel=args.channel, interface=fleet_config.BUSTYPE)
    pos, start = {}, {}
    print("move one joint by hand; the node whose numbers change is its drive. Ctrl-C ends.")
    try:
        t_next = time.monotonic()
        while True:
            m = bus.recv(0.02)
            if m is not None and (m.arbitration_id & 0x1F) == CMD_ENCODER and len(m.data) == 8:
                node = m.arbitration_id >> 5
                pos[node] = struct.unpack("<ff", bytes(m.data))[0]
                start.setdefault(node, pos[node])
            if time.monotonic() >= t_next:
                t_next += 0.2
                print("  ".join(f"node {n}: {pos[n] - start[n]:+7.3f}" for n in sorted(pos)),
                      end="\r", flush=True)
    except KeyboardInterrupt:
        print()
    finally:
        bus.shutdown()


async def cmd_backup(args):
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    async with Fleet(args.channel) as fleet:
        for sn, node in sorted(fleet.nodes().items(), key=lambda kv: kv[1]):
            cfg, info = await backup_one(fleet, sn)
            path = out / f"{joint_of(node)}_{sn}.json"
            path.write_text(json.dumps({"serial": sn, "node_id": node, "firmware": fw(info),
                                        "config": cfg}, indent=1) + "\n")
            print(f"  node {node:2d} {sn} fw {fw(info)}: {len(cfg)} settings -> {path}")


def load_backup(path: str) -> dict:
    d = json.loads(Path(path).read_text())
    return d["config"] if "config" in d else d          # ours, or odrivetool backup-config's


async def cmd_clone(args):
    node_id = args.node_id
    if node_id not in {c.node_id for c in fleet_config.OD_JOINTS}:
        sys.exit(f"node {node_id} isn't an ODrive joint in fleet_config.OD_JOINTS")
    async with Fleet(args.channel) as fleet:
        drives = fleet.nodes()
        print("drives on the bus: " + ", ".join(f"{sn}@{n}" for sn, n in drives.items()))
        owner = node_owner(drives, node_id, args.to)
        if owner:
            sys.exit(f"node {node_id} is already drive {owner}: refusing (both would obey it)")
        # The target first: if it shares its id with the source (a replaced board set to
        # the same id), connecting moves the one connected to a free id. Better the target.
        target, tinfo = await backup_one(fleet, args.to)
        if args.source.endswith(".json"):
            source, src_name = load_backup(args.source), args.source
        else:
            src_sn = args.source if args.source in drives else fleet.serial_at(int(args.source))
            if src_sn == args.to:
                sys.exit("source and target are the same drive")
            source, info = await backup_one(fleet, src_sn)
            src_name = f"{src_sn} (node {drives[src_sn]}, fw {fw(info)})"
        print(f"source {src_name}\ntarget {args.to} (now node {drives[args.to]}, "
              f"fw {fw(tinfo)}) -> node {node_id} = {joint_of(node_id)}")
        if tinfo["axis0.current_state"] != AXIS_STATE_IDLE:
            sys.exit(f"target not idle (state {tinfo['axis0.current_state']})")

        plan = plan_clone(source, target, node_id)
        print(f"\nwrites ({len(plan['writes'])}):")
        for k, v in plan["writes"].items():
            print(f"  {k:58s} {str(target.get(k))[:14]:>14s} -> {v}")
        if plan["kept"]:
            print("\nkept, the target's own calibration results:")
            for k, (t, s) in plan["kept"].items():
                print(f"  {k:58s} {t}   (source: {s})")
        if plan["missing"]:
            print(f"\nnot on the target's firmware, skipped: {', '.join(plan['missing'])}")
        if not args.write:
            print("\ndry run: nothing written. Add --write to apply and save.")
            return

        await write_plan(fleet, args.to, plan, node_id)
    await save_and_verify(args.channel, args.to, node_id)


async def write_plan(fleet: Fleet, serial: str, plan: dict, node_id: int) -> None:
    """Write the settings (verified by read-back), then the node id, which takes effect at
    once (the connection may drop). Nothing is saved yet: a power-off undoes it all."""
    from odrive.legacy_config import apply_config
    dev = await fleet.connect(serial)
    settings = [(k, v) for k, v in plan["writes"].items() if k != NODE_KEY]
    errors = await apply_config(dev, settings, verify=True) if settings else []
    if errors:
        fleet.dm.release_connection(dev)
        sys.exit("not saved; the drive keeps them only until power-off:\n  "
                 + "\n  ".join(errors))
    if settings:
        print(f"\nwrote {len(settings)} settings, read back OK")
    if NODE_KEY in plan["writes"]:
        try:
            await dev.write(dev.try_get_prop_info(NODE_KEY), node_id)
        except Exception:        # noqa: BLE001
            pass
    fleet.dm.release_connection(dev)


async def save_and_verify(channel: str, serial: str, node_id: int) -> None:
    """save_configuration over CAN (Reboot action 1) to the drive now on node_id, then
    check it comes back from the reboot on node_id, by serial."""
    hb = heartbeats(channel, 1.0)
    if node_id not in hb:
        sys.exit(f"no heartbeat at node {node_id} after the write ({sorted(hb)}): not saved; "
                 "power-cycling the drive drops everything written")
    if hb[node_id][1] != AXIS_STATE_IDLE:
        sys.exit(f"node {node_id} not idle: not saved")
    save_over_can(channel, node_id)
    print(f"saved over CAN (Reboot/save_configuration to node {node_id}); rebooting ...")
    time.sleep(3.0)
    hb = heartbeats(channel, 1.5)
    print("heartbeats now: " + ", ".join(f"node {n} ({r:.0f}/s)" for n, (_, _, _, r)
                                         in sorted(hb.items())))
    async with Fleet(channel) as fleet:
        if fleet.nodes().get(serial) != node_id:
            sys.exit(f"after the reboot drive {serial} is at {fleet.nodes().get(serial)}, "
                     f"not {node_id}: the save didn't take")
        dev = await fleet.connect(serial)
        try:
            back = await fleet.read(dev, *SPOT_CHECK)
        finally:
            fleet.dm.release_connection(dev)
    print(f"after reboot, drive {serial} is node {node_id}; spot check: {back}")
    print(f"next: python3 tools/odrive_check.py --joint {joint_of(node_id)} --repeat 3")


async def cmd_pins(args):
    """Which of the drive's pins change while you turn its joint, and the encoder count.
    An incremental encoder's A and B lines flip constantly while the motor turns (20000
    counts per motor turn), so they should toggle here even sampled at ~2 kHz. Compare a
    working drive (which pins are A/B) with the suspect one."""
    async with Fleet(args.channel) as fleet:
        sn = args.drive if args.drive in fleet.found else fleet.serial_at(int(args.drive))
        dev = await fleet.connect(sn)
        try:
            raw0 = (await fleet.read(dev, "inc_encoder0.raw"))["inc_encoder0.raw"]
            print(f"drive {sn} (node {fleet.nodes()[sn]}): TURN ITS JOINT back and forth "
                  f"now, for {args.secs:.0f} s ...", flush=True)
            toggles, hi, lo, prev, n = [0] * 16, 0, 0, None, 0
            t_end = time.monotonic() + args.secs
            while time.monotonic() < t_end:
                g = int(await dev.call_function("get_gpio_states"))
                n += 1
                hi, lo = hi | g, lo | ~g
                if prev is not None:
                    for b in range(16):
                        toggles[b] += ((g ^ prev) >> b) & 1
                prev = g
            raw1 = (await fleet.read(dev, "inc_encoder0.raw"))["inc_encoder0.raw"]
        finally:
            fleet.dm.release_connection(dev)
    print(f"{n} samples ({n / args.secs:.0f}/s)")
    for b in range(16):
        level = ("toggled %d times" % toggles[b] if toggles[b]
                 else "always HIGH" if hi >> b & 1 else "always low")
        if toggles[b] or hi >> b & 1:
            print(f"  gpio{b}: {level}")
    print(f"  (all other pins always low)\nencoder count: {raw0} -> {raw1}")
    toggled = [b for b in range(16) if toggles[b]]
    if raw1 != raw0:
        print("VERDICT: the encoder counts. Its signal reaches the drive.")
    elif toggled:
        print(f"VERDICT: pins {toggled} toggle but the count doesn't move: the signal arrives "
              "but not on the pins the counter uses. Compare with a working drive's pins.")
    else:
        print("VERDICT: no pin moved: no encoder signal reaches this board (encoder supply, "
              "cable, connector, or the encoder). Did the joint really turn?")


async def cmd_set_node(args):
    """Only the node id: for a drive whose settings are right but whose id is wrong."""
    node_id = args.node_id
    if node_id not in {c.node_id for c in fleet_config.OD_JOINTS}:
        sys.exit(f"node {node_id} isn't an ODrive joint in fleet_config.OD_JOINTS")
    async with Fleet(args.channel) as fleet:
        drives = fleet.nodes()
        print("drives on the bus: " + ", ".join(f"{sn}@{n}" for sn, n in drives.items()))
        if args.serial not in drives:
            sys.exit(f"drive {args.serial} not found")
        owner = node_owner(drives, node_id, args.serial)
        if owner:
            sys.exit(f"node {node_id} is already drive {owner}: refusing (both would obey it)")
        cfg, info = await backup_one(fleet, args.serial)
        if info["axis0.current_state"] != AXIS_STATE_IDLE:
            sys.exit(f"drive not idle (state {info['axis0.current_state']})")
        print(f"drive {args.serial} (fw {fw(info)}): node {drives[args.serial]} "
              f"(saved setting {cfg[NODE_KEY]}) -> {node_id} = {joint_of(node_id)}")
        if not args.write:
            print("dry run: nothing written. Add --write to set it and save.")
            return
        await write_plan(fleet, args.serial, {"writes": {NODE_KEY: node_id}}, node_id)
    await save_and_verify(args.channel, args.serial, node_id)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--channel", default=fleet_config.OD_CHANNEL)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list")
    sub.add_parser("watch")
    b = sub.add_parser("backup")
    b.add_argument("--out", default=str(CONFIG_DIR))
    c = sub.add_parser("clone")
    c.add_argument("--from", dest="source", required=True,
                   help="a working drive of the same kind (node id or serial), "
                        "or a backup .json")
    c.add_argument("--to", required=True, help="serial number of the target drive (list)")
    c.add_argument("--node-id", type=int, required=True, help="the target's node id")
    c.add_argument("--write", action="store_true", help="apply and save (default: dry run)")
    g = sub.add_parser("pins")
    g.add_argument("--drive", required=True, help="serial number or node id")
    g.add_argument("--secs", type=float, default=10.0)
    n = sub.add_parser("set-node")
    n.add_argument("--serial", required=True, help="the drive (serial number from list)")
    n.add_argument("--node-id", type=int, required=True)
    n.add_argument("--write", action="store_true", help="set and save (default: dry run)")
    args = ap.parse_args()

    ctl = ipc.CtlClient()
    try:
        ctl.request("status", timeout_s=0.3)
        sys.exit("a service answers on ctl (motor_service?): stop it first")
    except TimeoutError:
        pass
    finally:
        ctl.close()

    if args.cmd == "watch":
        cmd_watch(args)
    else:
        asyncio.run({"list": cmd_list, "backup": cmd_backup, "clone": cmd_clone,
                     "set-node": cmd_set_node, "pins": cmd_pins}[args.cmd](args))


if __name__ == "__main__":
    main()
