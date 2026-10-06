import sys, tempfile, threading, unittest
from datetime import timedelta
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import ApiError, DroneAirspaceService, iso, utcnow


class CorridorFlowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.svc = DroneAirspaceService(Path(self.tmp.name) / "test.db")
        self.start = utcnow() + timedelta(hours=2)
        # 三个协调区，沿经度 116.0 -> 116.6 顺序排列，容量各为 1
        self.zones = []
        for i, (lo, hi) in enumerate([(116.0, 116.2), (116.2, 116.4), (116.4, 116.6)], start=1):
            self.zones.append(self.svc.create_zone("rev", "airspace_reviewer", {
                "code": f"Z{i}", "name": f"区{i}", "order_index": i,
                "min_lon": lo, "min_lat": 39.7, "max_lon": hi, "max_lat": 40.0, "capacity": 1}))

    def tearDown(self):
        self.tmp.cleanup()

    def plan(self, callsign="D100", route=None, hours=2):
        route = route or [[116.05, 39.8], [116.15, 39.8], [116.25, 39.8], [116.35, 39.8], [116.45, 39.8], [116.55, 39.8]]
        p = self.svc.create_plan("op-user", "operator", "OP1", {
            "callsign": callsign, "drone_model": "M400", "payload_kg": 5, "route": route,
            "starts_at": iso(self.start + timedelta(hours=hours)), "ends_at": iso(self.start + timedelta(hours=hours + 1)),
            "max_altitude": 100, "population_risk": 1, "emergency_plan": "返回起降点", "region": "BJ"})
        self.svc.submit(p["id"], "op-user", "operator", "OP1", {})
        self.svc.approve(p["id"], "rev", "airspace_reviewer", {"expected_revision": 1, "offline_id": f"appr-{callsign}", "reason": "满足要求"})
        return p

    def test_segments_created_on_approval_and_sequential_handover(self):
        p = self.plan()
        state = self.svc.corridor_state(p["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "active")
        self.assertEqual([(s["sequence"], s["zone"]["code"], s["status"]) for s in state["segments"]],
                         [(1, "Z1", "pending"), (2, "Z2", "pending"), (3, "Z3", "pending")])
        # 逐区交接：接收 -> 放行 -> 下一区接收
        for seq in (1, 2, 3):
            acc = self.svc.accept_segment(p["id"], seq, "rev", "airspace_reviewer", {"offline_id": f"acc-{seq}"})
            self.assertEqual(acc["segment"]["status"], "accepted")
            rel = self.svc.release_segment(p["id"], seq, "rev", "airspace_reviewer", {"offline_id": f"rel-{seq}"})
            self.assertEqual(rel["segment"]["status"], "released")
        state = self.svc.corridor_state(p["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "completed")
        self.assertIsNone(state["current_position"])

    def test_next_zone_requires_previous_release_and_latecomer_sees_current_zone(self):
        p = self.plan()
        self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "acc-1"})
        with self.assertRaises(ApiError) as ctx:
            self.svc.accept_segment(p["id"], 2, "rev", "airspace_reviewer", {"offline_id": "acc-2"})
        self.assertEqual(ctx.exception.code, "previous_not_released")
        self.assertEqual(ctx.exception.details["current_position"]["zone"]["code"], "Z1")
        self.assertEqual(ctx.exception.details["target_zone"]["code"], "Z2")

    def test_restriction_change_invalidates_downstream_vouchers(self):
        p = self.plan()
        self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "acc-1"})
        self.svc.release_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "rel-1"})
        self.svc.accept_segment(p["id"], 2, "rev", "airspace_reviewer", {"offline_id": "acc-2"})
        # Z2 限制变化 -> Z3 未确认凭证作废，退回重排
        self.svc.create_restriction("rev", "airspace_reviewer", {
            "name": "临时活动", "kind": "temporary_limit", "zone_id": self.zones[1]["id"],
            "min_lon": 116.2, "min_lat": 39.7, "max_lon": 116.4, "max_lat": 40.0,
            "min_altitude": 0, "max_altitude": 150,
            "starts_at": iso(self.start - timedelta(minutes=30)), "ends_at": iso(self.start + timedelta(hours=2)),
            "reason": "活动"})
        state = self.svc.corridor_state(p["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "rearranging")
        self.assertEqual(state["segments"][2]["status"], "invalidated")
        # Z2 已接收，仍在 Z2；Z1 已放行
        self.assertEqual(state["segments"][1]["status"], "accepted")
        notifs = self.svc.notifications("rev", "airspace_reviewer", "")["notifications"]
        self.assertEqual(notifs[0]["kind"], "corridor_rearrange")
        # 退回重排后 Z3 可重新确认
        rr = self.svc.rearrange(p["id"], "rev", "airspace_reviewer")
        self.assertEqual(rr["rearranged"], 1)
        state = self.svc.corridor_state(p["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "active")
        self.assertEqual(state["segments"][2]["status"], "pending")

    def test_offline_idempotency_merges_by_zone(self):
        p = self.plan()
        first = self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "off-1"})
        self.assertFalse(first["idempotent"])
        again = self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "off-1"})
        self.assertTrue(again["idempotent"])
        # 同一离线编号用于另一区 -> 冲突
        with self.assertRaises(ApiError) as ctx:
            self.svc.accept_segment(p["id"], 2, "rev", "airspace_reviewer", {"offline_id": "off-1"})
        self.assertEqual(ctx.exception.code, "offline_id_conflict")

    def test_capacity_full_and_cross_zone_occupancy(self):
        a = self.plan("D200", hours=2)
        b = self.plan("D201", route=[[116.05, 39.8], [116.15, 39.8]], hours=5)
        self.svc.accept_segment(a["id"], 1, "rev", "airspace_reviewer", {"offline_id": "a-1"})
        # Z1 容量 1 已满
        with self.assertRaises(ApiError) as ctx:
            self.svc.accept_segment(b["id"], 1, "rev", "airspace_reviewer", {"offline_id": "b-1"})
        self.assertEqual(ctx.exception.code, "capacity_full")
        # 同一计划不能同时占两区
        with self.assertRaises(ApiError) as ctx:
            self.svc.accept_segment(a["id"], 2, "rev", "airspace_reviewer", {"offline_id": "a-2"})
        self.assertIn(ctx.exception.code, {"previous_not_released", "plan_in_zone"})

    def test_concurrent_accept_first_write_wins_latecomer_sees_zone(self):
        p = self.plan()
        results = []
        def race(oid):
            try:
                r = self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": oid})
                results.append(("ok", oid, r["idempotent"]))
            except ApiError as e:
                results.append(("err", oid, e.code, e.details.get("current_position", {}).get("zone", {}).get("code")))
        t1 = threading.Thread(target=race, args=("race-1",))
        t2 = threading.Thread(target=race, args=("race-2",))
        t1.start(); t2.start(); t1.join(); t2.join()
        oks = [r for r in results if r[0] == "ok"]
        errs = [r for r in results if r[0] == "err"]
        self.assertEqual(len(oks), 1)
        self.assertEqual(len(errs), 1)
        self.assertEqual(errs[0][3], "Z1")

    def test_backfill_legacy_plans_and_manual_for_undeterminable(self):
        # 旧计划：批准后删除区段并置为 none，模拟没有区段的旧数据
        legacy = self.plan("D300", route=[[116.05, 39.8], [116.55, 39.8]], hours=2)
        self.svc.repo.conn.execute("DELETE FROM plan_segments WHERE plan_id=?", (legacy["id"],))
        self.svc.repo.conn.execute("UPDATE flight_plans SET corridor_status='none' WHERE id=?", (legacy["id"],))
        # 航线完全在走廊外 -> 无法判断，交人工
        outside = self.plan("D301", route=[[117.0, 39.8], [117.1, 39.8]], hours=5)
        state = self.svc.corridor_state(outside["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "manual")
        self.assertEqual(state["corridor_note"], "route_outside_corridor")
        bf = self.svc.backfill_all("rev", "airspace_reviewer")
        backfilled_ids = {b["plan_id"] for b in bf["backfilled"]}
        self.assertIn(legacy["id"], backfilled_ids)
        manual_ids = {m["plan_id"] for m in bf["manual"]}
        self.assertIn(outside["id"], manual_ids)
        # 人工指定区段后可继续交接
        self.svc.manual_assign(outside["id"], "rev", "airspace_reviewer", {"zone_ids": [self.zones[0]["id"], self.zones[1]["id"]]})
        state = self.svc.corridor_state(outside["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "active")
        self.assertEqual(len(state["segments"]), 2)

    def test_restart_derives_occupancy_and_no_duplicate_notification(self):
        p = self.plan()
        self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "acc-1"})
        self.svc.release_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "rel-1"})
        self.svc.accept_segment(p["id"], 2, "rev", "airspace_reviewer", {"offline_id": "acc-2"})
        # 重新实例化服务（模拟重启）
        svc2 = DroneAirspaceService(Path(self.tmp.name) / "test.db")
        zones = svc2.list_zones("airspace_reviewer")["zones"]
        self.assertEqual({z["code"]: z["occupied"] for z in zones}, {"Z1": 0, "Z2": 1, "Z3": 0})
        # 通知去重：同一 (plan, kind, key) 只插入一次
        before = len(svc2.notifications("rev", "airspace_reviewer", "")["notifications"])
        with svc2.repo.tx() as conn:
            from app import Repository
            Repository.notify_once(conn, p["id"], "segment_accepted", "dup", f"accept:{p['id']}:{self.zones[1]['id']}:2")
            Repository.notify_once(conn, p["id"], "segment_accepted", "dup", f"accept:{p['id']}:{self.zones[1]['id']}:2")
        after = len(svc2.notifications("rev", "airspace_reviewer", "")["notifications"])
        self.assertEqual(before, after)

    def test_plan_change_invalidates_vouchers(self):
        p = self.plan()
        self.svc.accept_segment(p["id"], 1, "rev", "airspace_reviewer", {"offline_id": "acc-1"})
        changed = self.svc.change(p["id"], "op-user", "operator", "OP1", {"expected_revision": 1})
        self.assertEqual(changed["revision"], 2)
        state = self.svc.corridor_state(p["id"], "airspace_reviewer", "")
        self.assertEqual(state["corridor_status"], "rearranging")
        self.assertEqual(state["segments"][0]["status"], "invalidated")


if __name__ == "__main__":
    unittest.main()
