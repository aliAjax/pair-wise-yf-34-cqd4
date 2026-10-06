import sys, tempfile, threading, unittest
from datetime import timedelta
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app import ApiError, DroneAirspaceService, iso, utcnow


class ZoneFlowTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.db = Path(self.tmp.name) / "test.db"
        self.svc = DroneAirspaceService(self.db)
        self.start = utcnow() + timedelta(hours=2)
        # 三个东西向并排、互不重叠的协调区，走廊 A -> B -> C
        for code, name, seq, lon0, lon1, cap in (
            ("Z-A", "起点协调区", 1, 116.0, 116.4, 1),
            ("Z-B", "中途责任区", 2, 116.4, 116.8, 1),
            ("Z-C", "终点协调区", 3, 116.8, 117.2, 2),
        ):
            self.svc.create_zone("staff", "airspace_reviewer",
                                 {"code": code, "name": name, "seq": seq, "min_lon": lon0, "min_lat": 39.7,
                                  "max_lon": lon1, "max_lat": 40.1, "capacity": cap})

    def tearDown(self): self.tmp.cleanup()

    def plan(self, callsign="Z100", route=None, start=None):
        return self.svc.create_plan("op-user", "operator", "OP1", {
            "callsign": callsign, "drone_model": "M400", "payload_kg": 5,
            "route": route or [[116.1, 39.9], [116.4, 39.9], [116.7, 39.9], [117.0, 39.9]],
            "starts_at": iso(start or self.start), "ends_at": iso((start or self.start) + timedelta(hours=1)),
            "max_altitude": 100, "population_risk": 1, "emergency_plan": "返回起降点", "region": "CORRIDOR"})

    def accept(self, pid, seq, zone, actor=None, offline=None, role="airspace_reviewer"):
        return self.svc.accept_segment(pid, seq, actor or f"duty-{zone}", role, zone,
                                       {"offline_id": offline or f"{zone}-a-{pid}-{seq}", "note": "按当时限制与容量接收"})

    def release(self, pid, seq, zone, actor=None, offline=None, role="airspace_reviewer"):
        return self.svc.release_segment(pid, seq, actor or f"duty-{zone}", role, zone,
                                        {"offline_id": offline or f"{zone}-r-{pid}-{seq}"})

    # ------------------------------------------------------------ 1. 顺序闸门与凭证

    def test_sequential_zone_handoff_with_version_bound_vouchers(self):
        plan = self.plan()
        out = self.svc.submit(plan["id"], "op-user", "operator", "OP1", {})
        self.assertFalse(out["corridor"]["manual"])
        segs = out["plan"]["segments"]
        self.assertEqual([s["zone_code"] for s in segs], ["Z-A", "Z-B", "Z-C"])
        self.assertEqual(out["plan"]["corridor"], {"status": "flowing", "current_zone": "Z-A", "gate_seq": 1, "released_seqs": []})

        # 上一区未放行，B 区不能提前确认
        with self.assertRaises(ApiError) as ctx:
            self.accept(plan["id"], 2, "Z-B")
        self.assertEqual(ctx.exception.code, "out_of_order")
        self.assertEqual(ctx.exception.details["current_zone"], "Z-A")

        v1 = self.accept(plan["id"], 1, "Z-A")
        self.assertEqual(v1["voucher_id"], f"V-P{plan['id']}-R1-S1-Z-A")
        self.assertEqual(v1["bound_revision"], 1)
        # A 只接收未放行时，B 仍不能确认
        with self.assertRaises(ApiError) as ctx:
            self.accept(plan["id"], 2, "Z-B")
        self.assertEqual(ctx.exception.code, "out_of_order")

        self.release(plan["id"], 1, "Z-A")
        self.accept(plan["id"], 2, "Z-B")
        self.release(plan["id"], 2, "Z-B")
        # 值班员只能操作本协调区
        with self.assertRaises(ApiError) as ctx:
            self.accept(plan["id"], 3, "Z-B")
        self.assertEqual(ctx.exception.code, "zone_mismatch")
        self.accept(plan["id"], 3, "Z-C")
        done = self.release(plan["id"], 3, "Z-C")
        self.assertTrue(done["corridor_completed"])
        final = self.svc.get_plan(plan["id"], "airspace_reviewer", "")
        self.assertEqual(final["status"], "approved")
        self.assertEqual([s["status"] for s in final["segments"]], ["released", "released", "released"])

    def test_two_operators_confirming_same_segment_first_write_wins(self):
        p1 = self.plan("Z200"); p2 = self.plan("Z201")
        self.svc.submit(p1["id"], "op-user", "operator", "OP1", {})
        self.svc.submit(p2["id"], "op-user", "operator", "OP1", {})
        # C 区容量为 2，但本测试关注的是同一计划同一槽位的并发
        winner = {}
        errors = []

        def both_accept_a(pid, who):
            try:
                res = self.accept(pid, 1, "Z-A", actor=who, offline=f"A-{pid}-{who}")
                winner.setdefault("ok", res)
            except ApiError as exc:
                errors.append(exc)

        t1 = threading.Thread(target=both_accept_a, args=(p1["id"], "duty-A-1"))
        t2 = threading.Thread(target=both_accept_a, args=(p1["id"], "duty-A-2"))
        t1.start(); t2.start(); t1.join(); t2.join()
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0].code, "handoff_conflict")
        self.assertEqual(errors[0].details["voucher_id"], winner["ok"]["voucher_id"])
        self.assertEqual(errors[0].details["current_zone"], "Z-A")
        # 后到者重放自己的编号：冲突依然可查，但不会再占位
        with self.assertRaises(ApiError) as ctx:
            self.accept(p1["id"], 1, "Z-A", actor="duty-A-2", offline=f"A-{p1['id']}-duty-A-2")
        self.assertEqual(ctx.exception.code, "handoff_conflict")

    # ------------------------------------------------------------ 2. 限制变化作废旧凭证

    def test_restriction_change_voids_later_unconfirmed_segments(self):
        plan = self.plan()
        self.svc.submit(plan["id"], "op-user", "operator", "OP1", {})
        self.accept(plan["id"], 1, "Z-A"); self.release(plan["id"], 1, "Z-A")
        self.accept(plan["id"], 2, "Z-B")
        # B 区内新增临时禁飞，覆盖 B 段时间窗
        created = self.svc.create_restriction("reviewer", "airspace_reviewer", {
            "name": "B区临时管制", "kind": "no_fly", "min_lon": 116.5, "min_lat": 39.8, "max_lon": 116.6, "max_lat": 40.0,
            "min_altitude": 0, "max_altitude": 150,
            "starts_at": iso(self.start + timedelta(minutes=10)), "ends_at": iso(self.start + timedelta(minutes=50)),
            "reason": "临时活动"})
        self.assertEqual(created["voided_segments"], [{"plan_id": plan["id"], "revision": 1, "voided_seqs": [3]}])
        plan_view = self.svc.get_plan(plan["id"], "operator", "OP1")
        self.assertEqual(plan_view["status"], "rescheduling")
        self.assertEqual(plan_view["corridor"]["status"], "voided")
        self.assertEqual(plan_view["corridor"]["gate_seq"], 2)
        self.assertEqual(plan_view["corridor"]["current_zone"], "Z-B")
        self.assertEqual(plan_view["segments"][0]["status"], "released")  # 已放行历史保留
        # B 的已确认凭证保留快照，且标记与当前限制不一致
        self.assertTrue(plan_view["segments"][1]["basis"])
        self.assertFalse(plan_view["segments"][1]["basis_current"])
        with self.assertRaises(ApiError) as ctx:
            self.release(plan["id"], 2, "Z-B")
        self.assertEqual(ctx.exception.code, "zone_restriction_changed")
        notes = [n["kind"] for n in self.svc.notifications("op-user", "operator", "OP1")["notifications"]]
        self.assertIn("segments_voided", notes)
        # 运营方换版重排后，走廊从 Z-A 重新开始
        changed = self.svc.change(plan["id"], "op-user", "operator", "OP1", {"expected_revision": 1})
        resubmitted = self.svc.submit(plan["id"], "op-user", "operator", "OP1", {})["plan"]
        self.assertEqual(resubmitted["revision"], 2)
        self.assertEqual(resubmitted["corridor"]["current_zone"], "Z-A")
        self.assertEqual([s["status"] for s in resubmitted["segments"]], ["pending", "pending", "pending"])

    def test_restriction_blocks_accept_under_current_rules_and_capacity(self):
        # 起点区先有限制：接收时按当时限制直接拒绝
        self.svc.create_restriction("reviewer", "airspace_reviewer", {
            "name": "A区禁飞", "kind": "no_fly", "min_lon": 116.05, "min_lat": 39.8, "max_lon": 116.2, "max_lat": 40.0,
            "min_altitude": 0, "max_altitude": 150,
            "starts_at": iso(self.start), "ends_at": iso(self.start + timedelta(hours=1)), "reason": "活动"})
        plan = self.plan()
        self.svc.submit(plan["id"], "op-user", "operator", "OP1", {})
        with self.assertRaises(ApiError) as ctx:
            self.accept(plan["id"], 1, "Z-A")
        self.assertEqual(ctx.exception.code, "zone_restriction")
        self.assertEqual(ctx.exception.details["zone"], "Z-A")

    def test_zone_capacity_not_cross_occupied_and_released_frees_slot(self):
        p1 = self.plan("Z300"); self.svc.submit(p1["id"], "op-user", "operator", "OP1", {})
        self.accept(p1["id"], 1, "Z-A")
        # 另一个时间窗重叠、同走廊计划：A/B 容量 1，应被 A 区占位挡住
        p2 = self.plan("Z301", start=self.start + timedelta(minutes=10))
        self.svc.submit(p2["id"], "op-user", "operator", "OP1", {})
        with self.assertRaises(ApiError) as ctx:
            self.accept(p2["id"], 1, "Z-A", offline="A-p2-blocked")
        self.assertEqual(ctx.exception.code, "capacity_full")
        self.assertEqual(len(ctx.exception.details["occupants"]), 1)
        # A 放行后容量释放（released 不再占位），但 B 仍被 p1 的 B 段挡住：先放行 p1 的 A
        self.release(p1["id"], 1, "Z-A")
        self.accept(p2["id"], 1, "Z-A", offline="A-p2-ok"); self.release(p2["id"], 1, "Z-A")
        self.accept(p1["id"], 2, "Z-B")
        with self.assertRaises(ApiError) as ctx:
            self.accept(p2["id"], 2, "Z-B", offline="B-p2-blocked")
        self.assertEqual(ctx.exception.code, "capacity_full")
        # C 容量 2，p1 可正常接收，不跨区占用 B 的名额
        self.release(p1["id"], 2, "Z-B")
        self.accept(p1["id"], 3, "Z-C")

    # ------------------------------------------------------------ 3. 断线回传 / 重启

    def test_offline_sync_merges_per_zone_and_resumes_only_unfinished(self):
        plan = self.plan()
        self.svc.submit(plan["id"], "op-user", "operator", "OP1", {})
        batch = {"items": [
            {"seq": 1, "action": "accept", "offline_id": "Z-A-batch-1"},
            {"seq": 1, "action": "release", "offline_id": "Z-A-batch-2"},
            {"seq": 2, "action": "accept", "offline_id": "Z-B-batch-1"},  # 顺序未到（A 刚放，B 可以接收，这里应为成功）
            {"seq": 3, "action": "accept", "offline_id": "Z-C-batch-1"},  # B 未放行 -> out_of_order
        ]}
        out = self.svc.sync_segments(plan["id"], "commander", "commander", None, batch)
        statuses = [(r["seq"], r["ok"], r.get("error")) for r in out["results"]]
        self.assertEqual(statuses, [(1, True, None), (1, True, None), (2, True, None), (3, False, "out_of_order")])
        self.assertEqual(out["finished"], [1, 1, 2])
        # 整体重放：已完成区段幂等返回，未完成的 C 继续失败但不产生副作用
        replay = self.svc.sync_segments(plan["id"], "commander", "commander", None, batch)
        self.assertTrue(all(r.get("idempotent") for r in replay["results"][:3]))
        view = self.svc.get_plan(plan["id"], "commander", "")
        self.assertEqual([s["status"] for s in view["segments"]], ["released", "accepted", "pending"])

    def test_offline_ids_scoped_per_zone(self):
        # 同一离线编号在两个区各自使用都合法（按区编号合并）
        p1 = self.plan("Z400"); self.svc.submit(p1["id"], "op-user", "operator", "OP1", {})
        self.accept(p1["id"], 1, "Z-A", offline="shared-offline-1"); self.release(p1["id"], 1, "Z-A")
        self.accept(p1["id"], 2, "Z-B", offline="shared-offline-1")
        # 但同一区内编号复用给别的计划 -> 冲突
        p2 = self.plan("Z401"); self.svc.submit(p2["id"], "op-user", "operator", "OP1", {})
        with self.assertRaises(ApiError) as ctx:
            self.accept(p2["id"], 1, "Z-A", offline="shared-offline-1")
        self.assertEqual(ctx.exception.code, "offline_zone_conflict")
        self.assertEqual(ctx.exception.details["plan_id"], p1["id"])

    def test_restart_does_not_duplicate_notifications_or_slots(self):
        plan = self.plan(); self.svc.submit(plan["id"], "op-user", "operator", "OP1", {})
        self.accept(plan["id"], 1, "Z-A", offline="A-restart")
        before = self.svc.notifications("op-user", "operator", "OP1")["notifications"]
        # 模拟服务重启：同一数据库重新实例化服务
        svc2 = DroneAirspaceService(self.db)
        again = svc2.accept_segment(plan["id"], 1, "duty-Z-A", "airspace_reviewer", "Z-A", {"offline_id": "A-restart"})
        self.assertTrue(again["idempotent"])
        after = svc2.notifications("op-user", "operator", "OP1")["notifications"]
        self.assertEqual(len(after), len(before))
        self.assertEqual(next(n for n in after if n["kind"] == "segment_accepted")["delivered"], 0)
        seg = svc2.get_plan(plan["id"], "airspace_reviewer", "")["segments"][0]
        self.assertEqual(seg["status"], "accepted")
        self.assertEqual(seg["accepted_by"], "duty-Z-A")

    # ------------------------------------------------------------ 4. 旧计划回填

    def test_backfill_legacy_plan_by_route_and_restriction_extent(self):
        # 直接建旧计划、不提交（模拟没有区段的存量数据）
        plan = self.plan("Z500")
        out = self.svc.backfill_segments(plan["id"], "staff", "airspace_reviewer", {})
        self.assertFalse(out["manual"])
        self.assertEqual([s["zone_code"] for s in out["segments"]], ["Z-A", "Z-B", "Z-C"])
        # 再次回填幂等
        again = self.svc.backfill_segments(plan["id"], "staff", "airspace_reviewer", {})
        self.assertTrue(again["idempotent"])

    def test_unjudgeable_plan_goes_manual(self):
        # 航线只穿过 A、C 之间的一条缝（这里让路线落在无区覆盖区域）：交人工
        plan = self.plan("Z501", route=[[115.7, 39.9], [115.9, 39.9]])
        out = self.svc.backfill_segments(plan["id"], "staff", "airspace_reviewer", {})
        self.assertTrue(out["manual"])
        state = self.svc.state("airspace_reviewer", "")
        self.assertEqual(len(state["manual_queue"]), 1)
        view = self.svc.get_plan(plan["id"], "airspace_reviewer", "")
        self.assertEqual(view["corridor"]["status"], "manual")
        self.assertEqual(view["manual_review"]["status"], "open")
        # 人工给出经过的区序列后按声明路径回填落地
        fixed = self.svc.backfill_segments(plan["id"], "staff", "commander", {"zones": ["Z-A", "Z-B"]})
        self.assertFalse(fixed["manual"])
        self.assertEqual([s["zone_code"] for s in fixed["segments"]], ["Z-A", "Z-B"])
        view = self.svc.get_plan(plan["id"], "airspace_reviewer", "")
        self.assertEqual(view["manual_review"]["status"], "resolved")

    def test_plan_change_renews_segments_and_releases_capacity(self):
        p1 = self.plan("Z600"); self.svc.submit(p1["id"], "op-user", "operator", "OP1", {})
        self.accept(p1["id"], 1, "Z-A"); self.release(p1["id"], 1, "Z-A")
        self.accept(p1["id"], 2, "Z-B")
        # 变更计划：版本 +1，B 的已接收凭证 superseded（释放容量），需重走走廊
        changed = self.svc.change(p1["id"], "op-user", "operator", "OP1", {"expected_revision": 1})
        self.assertEqual(changed["revision"], 2)
        self.assertEqual(changed["status"], "draft")
        resubmitted = self.svc.submit(p1["id"], "op-user", "operator", "OP1", {})["plan"]
        self.assertEqual([s["status"] for s in resubmitted["segments"]], ["pending", "pending", "pending"])
        # 旧版本区段是历史视图，且 B 的旧接收不再占容量：另一个计划可接收 B
        old = [s for s in self.svc.repo.conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND plan_revision=1 ORDER BY seq", (p1["id"],))]
        self.assertEqual([s["status"] for s in old], ["released", "superseded", "voided"])


if __name__ == "__main__": unittest.main()
