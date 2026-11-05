"""
test_imu_locate.py — imu_locate.py without hardware.

A fake Nicla answers 'L' like firmware v2 for a board mounted on the robot with a
chosen rotation, while the "robot" is put through the poses and tilts the tool asks
for. Every square mounting (24) has to come back exactly, survive the trip through
the URDF and MuJoCo, and give the policy the right gravity and turn direction through
imu_protocol, the same path as deployment. The files are written to copies in
tmp_path; the repo's own are never touched.
"""
from __future__ import annotations

import builtins
import importlib.util
import math
import shutil

import mujoco
import numpy as np
import pytest

import assets
import imu_locate as loc
import imu_protocol as proto
from rl_env import projected_gravity_from_quat
from sim_env import _build_model

REPO = loc.REPO
V2 = {"proto": 2, "accel_range_g": 4.0, "gyro_range_dps": 1000.0}  # as flashed
COUNTS_PER_G = 32768.0 / V2["accel_range_g"]
COUNTS_PER_DPS = 32768.0 / V2["gyro_range_dps"]
PERMS = loc.SIGNED_PERMUTATIONS


def rot_x(deg):
    return loc.matrix_from_rpy(math.radians(deg), 0, 0)


def rot_y(deg):
    return loc.matrix_from_rpy(0, math.radians(deg), 0)


def rot_z(deg):
    return loc.matrix_from_rpy(0, 0, math.radians(deg))


# Body -> world in each pose the tool asks for (+y pitches the nose down, -x drops
# the left side), with a little slop so nothing passes by being exact.
POSE_WORLD = {"level": rot_x(1.0) @ rot_y(-0.5),
              "nose_down": rot_y(85.0) @ rot_x(6.0),
              "left_down": rot_x(-95.0) @ rot_y(-4.0)}


def board_up(mount, body_to_world):
    """Unit UP vector in board axes; mount maps board -> body (= AXIS_REMAP)."""
    return (body_to_world @ mount).T @ np.array([0.0, 0.0, 1.0])


class FakeNicla:
    """Answers 'L' with a v2 'ALL' line for a board at `mount` on a robot whose
    body -> world rotation and body-frame angular velocity [rad/s] are set by the test."""

    def __init__(self, mount):
        self.mount = np.asarray(mount, dtype=float)
        self.world = np.eye(3)
        self.omega = np.zeros(3)
        self.step = None            # optional callable advancing the motion per poll
        self._line = b""

    def reset_input_buffer(self):
        pass

    def write(self, data):
        if data != b"L":
            self._line = b""
            return
        if self.step:
            self.step(self)
        b2w = self.world @ self.mount
        a = b2w.T @ np.array([0.0, 0.0, 1.0]) * COUNTS_PER_G
        w = self.mount.T @ np.degrees(self.omega) * COUNTS_PER_DPS
        q = proto._matrix_to_quat(b2w.tolist())
        fields = [*a, *w, 0.0, 0.0, *q]
        self._line = ("ALL " + " ".join(f"{v:.6f}" for v in fields) + "\r\n").encode()

    def readline(self):
        return self._line


def scripted_input(monkeypatch, actions):
    """Replace input(): each call runs the next action (or answers its string)."""
    it = iter(actions)

    def fake(prompt=""):
        a = next(it)
        if callable(a):
            a()
            return ""
        return a
    monkeypatch.setattr(builtins, "input", fake)


def use_remap(monkeypatch, r):
    """Point imu_protocol at a remap the way an import of a rewritten file would."""
    r = tuple(tuple(float(v) for v in row) for row in r)
    q = proto._matrix_to_quat(r)
    monkeypatch.setattr(proto, "AXIS_REMAP", r)
    monkeypatch.setattr(proto, "_Q_REMAP_CONJ", (q[0], -q[1], -q[2], -q[3]))


def files(tmp_path):
    """Copies of everything apply writes, laid out like dodo_assets so both models find
    their meshes. The URDF starts without an IMU block, whether or not the real one has
    one by now; the MJCF keeps its generated block, which apply replaces. Every call
    below passes these paths: a default would write into the real dodo_assets."""
    (tmp_path / "urdf").mkdir()
    (tmp_path / "urdf" / "dodo_daimao.urdf").write_text(
        loc._URDF_BLOCK_RE.sub("", loc.URDF_PATH.read_text()))
    (tmp_path / "mjcf").mkdir()
    shutil.copy(loc.MJCF_PATH, tmp_path / "mjcf" / "dodo_daimao.xml")
    (tmp_path / "meshes").symlink_to(assets.ASSETS_DIR / "meshes")
    shutil.copy(loc.PROTOCOL_PATH, tmp_path / "imu_protocol.py")
    shutil.copy(loc.README_PATH, tmp_path / "README.md")
    return dict(record_path=tmp_path / "imu_mount.json",
                urdf_path=tmp_path / "urdf" / "dodo_daimao.urdf",
                protocol_path=tmp_path / "imu_protocol.py",
                readme_path=tmp_path / "README.md",
                mjcf_path=tmp_path / "mjcf" / "dodo_daimao.xml")


def apply(rec, f):
    loc.apply_record(rec, f["urdf_path"], f["protocol_path"], f["readme_path"],
                     f["mjcf_path"])


def checks(f):
    return loc.run_checks(f["record_path"], f["urdf_path"], f["protocol_path"],
                          f["mjcf_path"])


def record_for(mount, xyz=(0.012, -0.003, 0.11)):
    return loc.make_record(mount, {"mode": "snapped"}, xyz, {"method": "given"})


class TestRotationMath:
    def test_24_square_mountings(self):
        assert len(PERMS) == 24
        assert all(loc.is_proper_rotation(p) for p in PERMS)

    @pytest.mark.parametrize("i", range(24))
    def test_rpy_round_trip_square(self, i):
        # Includes the gimbal-lock cases (chip x along body ±z).
        r = PERMS[i]
        assert loc.matrix_from_rpy(*loc.rpy_from_matrix(r)) == pytest.approx(r, abs=1e-9)

    def test_rpy_round_trip_random(self):
        rng = np.random.default_rng(0)
        for _ in range(200):
            r = loc.matrix_from_rpy(*rng.uniform(-math.pi, math.pi, 3))
            assert loc.matrix_from_rpy(*loc.rpy_from_matrix(r)) == pytest.approx(r, abs=1e-9)

    def test_rpy_is_urdf_extrinsic_xyz(self):
        # URDF: roll about fixed x first, then pitch about fixed y, then yaw about z.
        r = loc.matrix_from_rpy(0.3, 0.2, 0.1)
        assert r == pytest.approx(rot_z(math.degrees(0.1)) @ rot_y(math.degrees(0.2))
                                  @ rot_x(math.degrees(0.3)))

    def test_describe_axes(self):
        r = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])   # chip x -> right, y -> fwd
        assert loc.describe_axes(r) == "chip +x -> right, chip +y -> forward, chip +z -> up"


class TestSolveOrientation:
    @pytest.mark.parametrize("i", range(24))
    def test_every_square_mounting_comes_back(self, i):
        ups = {k: board_up(PERMS[i], w) for k, w in POSE_WORLD.items()}
        r, info, problems = loc.solve_orientation(ups)
        assert problems == []
        assert r == pytest.approx(PERMS[i])
        assert info["snap_gap_deg"] < 5 and info["crosscheck_deg"] < 15

    def test_nose_up_instead_of_nose_down_is_caught(self):
        ups = {k: board_up(PERMS[5], w) for k, w in POSE_WORLD.items()}
        ups["nose_down"] = board_up(PERMS[5], rot_y(-90))
        r, _, problems = loc.solve_orientation(ups)
        assert r is None and "wrong way" in problems[0]

    def test_right_side_down_instead_of_left_is_caught(self):
        ups = {k: board_up(PERMS[7], w) for k, w in POSE_WORLD.items()}
        ups["left_down"] = board_up(PERMS[7], rot_x(90))
        r, _, problems = loc.solve_orientation(ups)
        assert r is None and "wrong way" in problems[0]

    def test_skipped_pose_is_caught(self):
        ups = {k: board_up(PERMS[0], w) for k, w in POSE_WORLD.items()}
        ups["left_down"] = ups["level"]
        r, _, problems = loc.solve_orientation(ups)
        assert r is None and "~90" in problems[0]

    def test_board_at_an_angle_refuses_to_snap(self):
        mount = PERMS[3] @ rot_z(30)      # a square mount turned 30 deg on its plate
        ups = {k: board_up(mount, w) for k, w in POSE_WORLD.items()}
        _, _, problems = loc.solve_orientation(ups)
        assert problems and "--exact" in problems[0]
        r, info, problems = loc.solve_orientation(ups, exact=True)
        assert problems == [] and info["mode"] == "exact"
        assert loc.rotation_deg(r, mount) < 6     # limited by the pose slop


class TestMeasureOrientation:
    def run(self, monkeypatch, mount, worlds, answers=()):
        imu = FakeNicla(mount)

        def pose(w):
            return lambda: setattr(imu, "world", w)
        scripted_input(monkeypatch, [pose(w) for w in worlds] + list(answers))
        return loc.measure_orientation(imu, V2)

    @pytest.mark.parametrize("i", [0, 9, 17, 23])
    def test_through_the_serial_line(self, monkeypatch, i):
        got = self.run(monkeypatch, PERMS[i], POSE_WORLD.values())
        assert got is not None
        r, info = got
        assert r == pytest.approx(PERMS[i])
        assert info["firmware"] == 2 and set(info["up_board"]) == set(POSE_WORLD)

    def test_wrong_pose_offers_a_redo(self, monkeypatch):
        worlds = [POSE_WORLD["level"], rot_y(-90), POSE_WORLD["left_down"]]
        assert self.run(monkeypatch, PERMS[2], worlds, answers=["n"]) is None


class TestPosition:
    def test_offsets_to_body_frame(self):
        ax, az = loc.hip_pitch_axis(loc.URDF_PATH)
        assert (ax, az) == pytest.approx((-0.00075, 0.0))
        # 40 mm closer to the left feature than the right one: 20 mm left of centre.
        xyz = loc.position_from_offsets(loc.URDF_PATH, 30, 110, 80, 120)
        assert xyz == pytest.approx([0.03 - 0.00075, 0.02, 0.11])

    def test_outline_check(self):
        assert loc.position_warnings(loc.URDF_PATH, np.array([0.03, 0.0, 0.11])) == []
        warn = loc.position_warnings(loc.URDF_PATH, np.array([0.03, 0.0, -0.11]))
        assert warn and "outside" in warn[0]

    def test_arith(self):
        assert loc.arith("212 - 95") == 117
        assert loc.arith(" -(30 + 31) / 2 ") == -30.5
        for bad in ("__import__('os')", "x", "1 ** 2", "True", ""):
            with pytest.raises((ValueError, SyntaxError)):
                loc.arith(bad)


class TestApply:
    def test_files_agree_and_reapply_is_idempotent(self, tmp_path):
        f = files(tmp_path)
        rec = record_for(PERMS[11])
        loc.save_record(rec, f["record_path"])
        apply(rec, f)
        first = {k: p.read_text() for k, p in f.items()}
        apply(rec, f)
        assert {k: p.read_text() for k, p in f.items()} == first
        assert first["urdf_path"].count('<link name="imu_link">') == 1
        assert "_pending" not in first["readme_path"].split("## IMU")[1].split("##")[0]
        assert checks(f) == []

    def test_check_catches_a_stale_axis_remap(self, tmp_path):
        f = files(tmp_path)
        rec = record_for(PERMS[11])
        loc.save_record(rec, f["record_path"])
        loc.write_urdf(rec, f["urdf_path"])      # both models updated, imu_protocol.py not
        loc.write_mjcf(rec, f["mjcf_path"])
        assert checks(f) == ["AXIS_REMAP"]

    def test_check_catches_a_stale_mjcf(self, tmp_path):
        f = files(tmp_path)
        rec = record_for(PERMS[11])
        loc.save_record(rec, f["record_path"])
        loc.write_urdf(rec, f["urdf_path"])      # the MJCF still has the old mount
        loc.write_axis_remap(rec, f["protocol_path"])
        assert checks(f) == ["MJCF"]

    @pytest.mark.parametrize("i", [0, 7, 13, 21])
    def test_mjcf_and_urdf_compile_to_the_same_imu_frame(self, tmp_path, i):
        """What dodo_assets' check_models.py compares, at its 1e-9: an exact mount
        makes both models' imu_link quats agree to float noise, not to rounding."""
        f = files(tmp_path)
        rec = record_for(PERMS[i] @ rot_z(11) @ rot_x(-3))
        loc.write_urdf(rec, f["urdf_path"])
        loc.write_mjcf(rec, f["mjcf_path"])
        u = mujoco.MjModel.from_xml_path(str(f["urdf_path"]))
        m = mujoco.MjModel.from_xml_path(str(f["mjcf_path"]))
        qu, qm = u.body("imu_link").quat, m.body("imu_link").quat
        assert min(np.abs(qu - qm).max(), np.abs(qu + qm).max()) < 1e-9
        assert u.body("imu_link").pos == pytest.approx(m.body("imu_link").pos, abs=1e-12)
        # principal inertias: the importers list them in different orders
        assert np.sort(u.body("imu_link").inertia) == pytest.approx(
            np.sort(m.body("imu_link").inertia), rel=1e-12)

    def test_defaults_stay_in_the_urdfs_checkout(self, tmp_path):
        """Writing a copy of the URDF must never reach another checkout's MJCF or
        README: their defaults come from next to the URDF given."""
        f = files(tmp_path)
        real = {p: p.read_bytes() for p in (loc.MJCF_PATH, loc.README_PATH)}
        rec = record_for(PERMS[5])
        loc.save_record(rec, f["record_path"])
        loc.apply_record(rec, f["urdf_path"], f["protocol_path"])
        assert {p: p.read_bytes() for p in real} == real
        assert loc.run_checks(f["record_path"], f["urdf_path"], f["protocol_path"]) == []

    def test_mjcf_needs_its_markers(self, tmp_path):
        f = files(tmp_path)
        text = loc._MJCF_BLOCK_RE.sub('      <body name="imu_link"/>\n',
                                      f["mjcf_path"].read_text())
        f["mjcf_path"].write_text(text)
        with pytest.raises(ValueError, match="BEGIN/END"):
            loc.write_mjcf(record_for(PERMS[0]), f["mjcf_path"])

    @pytest.mark.parametrize("generated_too", [False, True])
    def test_refuses_to_duplicate_a_hand_written_imu(self, tmp_path, generated_too):
        f = files(tmp_path)
        if generated_too:
            loc.write_urdf(record_for(PERMS[0]), f["urdf_path"])
        text = f["urdf_path"].read_text().replace(
            "</robot>", '<link name="imu_link" />\n</robot>')
        f["urdf_path"].write_text(text)
        with pytest.raises(ValueError, match="hand-written"):
            loc.write_urdf(record_for(PERMS[0]), f["urdf_path"])

    def test_record_round_trip(self, tmp_path):
        rec = record_for(PERMS[0] @ rot_z(17))
        loc.save_record(rec, tmp_path / "r.json")
        assert loc.load_record(tmp_path / "r.json") == rec
        loc.save_record(record_for(PERMS[9]), tmp_path / "r.json")
        assert "[-1, 0, 0]" in (tmp_path / "r.json").read_text()   # one row per line

    def test_written_imu_protocol_gives_level_gravity(self, tmp_path):
        """The rewritten imu_protocol.py imports, and a level robot reads (0, 0, -1)
        through both the quaternion and the accelerometer path."""
        f = files(tmp_path)
        mount = PERMS[13]
        loc.write_axis_remap(record_for(mount), f["protocol_path"])
        spec = importlib.util.spec_from_file_location("written_protocol", f["protocol_path"])
        written = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(written)
        assert np.array(written.AXIS_REMAP) == pytest.approx(mount)

        imu = FakeNicla(mount)
        imu.write(b"L")
        g, a = 9.80665, 4.0 * 9.80665 / 32768.0
        r = written.parse_imu_line(imu.readline().decode(), 1.0, a)
        assert projected_gravity_from_quat(r["quat"]) == pytest.approx([0, 0, -1], abs=1e-5)
        assert np.array(r["accel"]) / g == pytest.approx([0, 0, 1], abs=1e-5)

    def test_exact_rotation_survives_the_files(self, tmp_path):
        f = files(tmp_path)
        mount = PERMS[4] @ rot_z(17) @ rot_x(-8)
        rec = record_for(mount)
        loc.save_record(rec, f["record_path"])
        apply(rec, f)
        assert loc.read_axis_remap(f["protocol_path"]) == pytest.approx(mount, abs=1e-8)
        assert checks(f) == []


class TestMuJoCo:
    def test_all_square_mountings_import_as_written(self, tmp_path):
        f = files(tmp_path)
        for p in PERMS:
            loc.write_urdf(record_for(p), f["urdf_path"])
            m = mujoco.MjSpec.from_file(str(f["urdf_path"])).compile()
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "imu_link")
            mat = np.zeros(9)
            mujoco.mju_quat2Mat(mat, m.body_quat[bid])
            assert mat.reshape(3, 3) == pytest.approx(p, abs=1e-9)
            assert m.body_pos[bid] == pytest.approx([0.012, -0.003, 0.11])

    def test_sim_model_unchanged_but_for_the_imu(self, tmp_path):
        f = files(tmp_path)
        before = _build_model(str(f["urdf_path"]))
        loc.write_urdf(record_for(PERMS[0]), f["urdf_path"])
        after = _build_model(str(f["urdf_path"]))
        assert (after.nq, after.nv, after.nu) == (before.nq, before.nv, before.nu)
        assert after.nbody == before.nbody + 1
        assert sum(after.body_mass) == pytest.approx(sum(before.body_mass) + loc.IMU_MASS_KG)


class TestLiveCheck:
    @staticmethod
    def motion(tilt, omega):
        """Tilt from level by 1.5 deg per poll to 45 deg, then hold. tilt(deg) is the
        body -> world rotation, omega the body-frame rate while it moves."""
        state = {"deg": 0.0}

        def step(imu):
            moving = state["deg"] < 45
            state["deg"] += 1.5 if moving else 0.0
            imu.omega = omega if moving else np.zeros(3)
            imu.world = tilt(state["deg"])
        return step

    def drive(self, monkeypatch, mount):
        imu = FakeNicla(mount)

        def level():
            imu.world, imu.omega, imu.step = np.eye(3), np.zeros(3), None

        def nose_down():   # +y rotation
            level()
            imu.step = self.motion(rot_y, np.array([0.0, 1.0, 0.0]))

        def left_down():   # -x rotation
            level()
            imu.step = self.motion(lambda d: rot_x(-d), np.array([-1.0, 0.0, 0.0]))
        scripted_input(monkeypatch, [level, nose_down, left_down])
        return loc.live_check(imu, V2, timeout_s=1.0)

    @pytest.mark.parametrize("i", [0, 6, 15, 22])
    def test_passes_with_the_right_remap(self, monkeypatch, i):
        use_remap(monkeypatch, PERMS[i])
        assert self.drive(monkeypatch, PERMS[i])

    def test_fails_with_a_wrong_remap(self, monkeypatch):
        use_remap(monkeypatch, np.eye(3))
        assert not self.drive(monkeypatch, PERMS[6])

    def test_level_catches_a_board_tilted_on_its_mount(self, monkeypatch):
        """The 2026-11-02 case: a board ~4.4 deg off square, snapped to the nearest
        square mount. The tilts still pass; only a strict level test can see that a
        level robot reads as tilted. (The old g_z < -0.97 test passed it.)"""
        snapped = PERMS[6]
        mount = snapped @ rot_x(3.0) @ rot_y(-3.2)
        use_remap(monkeypatch, snapped)
        assert not self.drive(monkeypatch, mount)
        use_remap(monkeypatch, mount)                  # the exact fit
        assert self.drive(monkeypatch, mount)


    def test_a_slow_tilt_says_rerun_not_remeasure(self, monkeypatch, capsys):
        """2026-11-04 on the robot: step 3 timed out with the robot barely rolled, and
        the tool said to re-measure the orientation. A tilt that never gets there is a
        re-run; only one that goes the wrong way points at the remap."""
        use_remap(monkeypatch, PERMS[6])
        imu = FakeNicla(PERMS[6])

        def level():
            imu.world, imu.omega, imu.step = np.eye(3), np.zeros(3), None

        def nose_down():
            level()
            imu.step = self.motion(rot_y, np.array([0.0, 1.0, 0.0]))
        scripted_input(monkeypatch, [level, nose_down, level])   # step 3: never rolled
        assert not loc.live_check(imu, V2, timeout_s=0.3)
        out = capsys.readouterr().out
        assert "it never got there" in out and "Re-run `imu_locate.py check --live`" in out
        assert "measure --skip-position" not in out

    @pytest.mark.parametrize("deg", [-6.0, 3.0])
    def test_tilt_parts_signs(self, deg):
        g_nose = rot_y(deg).T @ np.array([0.0, 0.0, -1.0])    # +y rotation: nose down
        g_right = rot_x(deg).T @ np.array([0.0, 0.0, -1.0])   # +x rotation: right down
        assert loc.tilt_parts(g_nose) == pytest.approx((abs(deg), deg, 0.0), abs=1e-6)
        assert loc.tilt_parts(g_right) == pytest.approx((abs(deg), 0.0, deg), abs=1e-6)

    def test_level_readout_shows_the_phantom_tilt(self, monkeypatch, capsys):
        """`level` prints what the policy sees: a level robot with a board ~4 deg off
        square reads that tilt under the snapped remap, and ~0 under the exact fit."""
        mount = PERMS[6] @ rot_x(3.0) @ rot_y(-3.2)
        down = np.array([0.0, 0.0, -1.0])
        phantom = loc.angle_deg(PERMS[6] @ mount.T @ down, down)   # what snapping leaves
        assert phantom > 2.0
        for remap, want in ((PERMS[6], phantom), (mount, 0.0)):
            use_remap(monkeypatch, remap)
            imu, polls = FakeNicla(mount), {"n": 0}

            def stop_after_a_while(_imu):
                polls["n"] += 1
                if polls["n"] > 40:
                    raise KeyboardInterrupt
            imu.step = stop_after_a_while
            monkeypatch.setattr(loc, "_open", lambda args: (imu, V2))
            assert loc.cmd_level(None) == 0
            last = capsys.readouterr().out.strip().split("\r")[-1]
            tilt = float(last.split("tilt")[1].split("deg")[0])
            assert tilt == pytest.approx(want, abs=0.1), last


class TestRefit:
    """`refit` re-solves from the record's stored poses: no IMU, no turning the robot."""
    MOUNT = PERMS[6] @ rot_x(3.0) @ rot_y(-3.2)        # ~4.4 deg off square

    def snapped_record(self):
        ups = {k: board_up(self.MOUNT, np.eye(3) if k == "level" else POSE_WORLD[k])
               for k in POSE_WORLD}
        r, info, problems = loc.solve_orientation(ups)
        assert not problems
        info.update(measured="2026-11-02", firmware=2)
        rec = loc.make_record(r, info, (0.0142, 0.0, 0.095),
                              {"method": "hand", "measured": "2026-11-02"},
                              note="steel rule", mass_kg=0.007)
        rec["_comment"] = "IMU mount on body."            # hand-edited, must survive
        return rec

    def test_exact_recovers_the_tilted_board(self):
        old = self.snapped_record()
        assert old["axis_remap"] == PERMS[6].astype(int).tolist()
        rec, problems = loc.refit_record(old, exact=True)
        assert not problems
        r = np.array(rec["axis_remap"])
        # Body z comes from the level pose alone (exact here); the heading from the
        # tilted poses, which POSE_WORLD leans sideways a few degrees on purpose.
        assert r[2] == pytest.approx(self.MOUNT[2], abs=1e-3)
        assert loc.rotation_deg(r, self.MOUNT) < 1.0
        o = rec["orientation"]
        assert o["mode"] == "exact" and o["level_tilt_deg"] == pytest.approx(0, abs=0.05)
        assert o["up_board"] == old["orientation"]["up_board"]     # raw data kept
        assert o["measured"] == "2026-11-02" and o["refit"]
        for k in ("_comment", "xyz_m", "position", "note", "mass_kg", "box_m"):
            assert rec[k] == old[k], k
        assert list(rec) == list(old)                     # same layout on disk
        back, _ = loc.refit_record(rec, exact=False)
        assert back["axis_remap"] == old["axis_remap"]

    def test_refit_record_goes_through_the_files(self, tmp_path):
        f = files(tmp_path)
        rec, _ = loc.refit_record(self.snapped_record(), exact=True)
        loc.save_record(rec, f["record_path"])
        apply(rec, f)
        assert loc.read_axis_remap(f["protocol_path"]) == pytest.approx(
            np.array(rec["axis_remap"]), abs=1e-8)
        assert checks(f) == []
