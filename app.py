#!/usr/bin/env python3
"""Drone flight-plan approval and zone-by-zone airspace coordination (standard library only)."""
from __future__ import annotations

import argparse
import json
import os
import sqlite3
import threading
from contextlib import contextmanager
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

PORT = 8205
ROLES = {"viewer", "operator", "airspace_reviewer", "commander", "auditor"}
ACTIVE_STATUSES = {"submitted", "approved"}
EPS = 1e-9

# 区段生命周期：pending 待确认 -> accepted 已接收(凭证有效) -> released 已放行；
# voided 作废（限制变化/计划换版），superseded 被新版本取代。
SEG_FLOW = ("pending", "accepted", "released", "voided", "superseded")


class ApiError(Exception):
    def __init__(self, status: int, code: str, message: str, details: Any = None):
        super().__init__(message); self.status, self.code, self.message, self.details = status, code, message, details


class ManualCorridor(Exception):
    """航线无法唯一映射到协调区，需要人工判定。"""


def utcnow() -> datetime: return datetime.now(timezone.utc)
def iso(value: datetime | None = None) -> str: return (value or utcnow()).replace(microsecond=0).isoformat().replace("+00:00", "Z")
def parse_time(value: str | None) -> datetime:
    if not value: raise ApiError(400, "time_required", "必须提供 ISO 8601 时间")
    try: parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc: raise ApiError(400, "invalid_time", f"时间格式错误: {value}") from exc
    return parsed.replace(tzinfo=timezone.utc) if parsed.tzinfo is None else parsed.astimezone(timezone.utc)


def route_bbox(route: list[list[float]]) -> tuple[float, float, float, float]:
    xs = [float(point[0]) for point in route]; ys = [float(point[1]) for point in route]
    return min(xs), min(ys), max(xs), max(ys)


def boxes_overlap(a: tuple[float, float, float, float], b: tuple[float, float, float, float], buffer: float = 0.0) -> bool:
    return a[0] <= b[2] + buffer and a[2] + buffer >= b[0] and a[1] <= b[3] + buffer and a[3] + buffer >= b[1]


def times_overlap(a_start: datetime, a_end: datetime, b_start: datetime, b_end: datetime) -> bool: return a_start < b_end and b_start < a_end


def validate_route(route: Any) -> list[list[float]]:
    if not isinstance(route, list) or len(route) < 2: raise ApiError(400, "invalid_route", "航线至少需要两个经纬度点")
    normalized: list[list[float]] = []
    for point in route:
        if not isinstance(point, list) or len(point) != 2 or not all(isinstance(v, (int, float)) for v in point): raise ApiError(400, "invalid_route_point", "每个航线点必须是 [经度,纬度]")
        lon, lat = float(point[0]), float(point[1])
        if not -180 <= lon <= 180 or not -90 <= lat <= 90: raise ApiError(400, "invalid_coordinates", "经纬度超出范围")
        normalized.append([lon, lat])
    return normalized


# ---------------------------------------------------------------- 航线切区

def _clip_edge(p0: list[float], p1: list[float], box: tuple[float, float, float, float]) -> tuple[float, float] | None:
    """Liang-Barsky：线段与矩形的交点参数 u∈[0,1] 区间，无交集返回 None。"""
    x0, y0, x1, y1 = box
    u0, u1, dx, dy = 0.0, 1.0, p1[0] - p0[0], p1[1] - p0[1]
    for p, q in ((-dx, p0[0] - x0), (dx, x1 - p0[0]), (-dy, p0[1] - y0), (dy, y1 - p0[1])):
        if abs(p) < EPS:
            if q < 0: return None
        else:
            r = q / p
            if p < 0: u0 = max(u0, r)
            else: u1 = min(u1, r)
            if u0 > u1: return None
    return (u0, u1) if u1 >= u0 else None


def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not intervals: return []
    intervals = sorted(intervals)
    merged = [list(intervals[0])]
    for a, b in intervals[1:]:
        if a <= merged[-1][1] + EPS: merged[-1][1] = max(merged[-1][1], b)
        else: merged.append([a, b])
    return [(a, b) for a, b in merged]


def _point_at(route: list[list[float]], t: float) -> list[float]:
    t = min(1.0, max(0.0, t)); span = len(route) - 1; f, i = min(1.0 - EPS, t * span), 0
    f = t * span; i = min(span - 1, int(f)); u = f - i
    return [route[i][0] + (route[i + 1][0] - route[i][0]) * u, route[i][1] + (route[i + 1][1] - route[i][1]) * u]


def _slice_points(route: list[list[float]], t0: float, t1: float) -> list[list[float]]:
    slice_pts = [_point_at(route, t0)]
    for i, point in enumerate(route):
        tv = i / (len(route) - 1)
        if t0 + EPS < tv < t1 - EPS: slice_pts.append(point)
    slice_pts.append(_point_at(route, t1))
    deduped = [slice_pts[0]]
    for pt in slice_pts[1:]:
        if abs(pt[0] - deduped[-1][0]) > EPS or abs(pt[1] - deduped[-1][1]) > EPS: deduped.append(pt)
    return deduped


def split_route_by_zones(route: list[list[float]], zones: list[sqlite3.Row]) -> list[dict[str, Any]]:
    """按协调区矩形把航线切成有序区段。覆盖不全、重叠、重回均交人工。"""
    if not zones: raise ManualCorridor("尚未定义任何协调区")
    per_zone: dict[int, list[tuple[float, float]]] = {z["id"]: [] for z in zones}
    edges = list(zip(route[:-1], route[1:]))
    for i, (p0, p1) in enumerate(edges):
        for z in zones:
            hit = _clip_edge(p0, p1, (z["min_lon"], z["min_lat"], z["max_lon"], z["max_lat"]))
            if hit: per_zone[z["id"]].append(((i + hit[0]) / (len(route) - 1), (i + hit[1]) / (len(route) - 1)))
    spans: list[tuple[float, float, sqlite3.Row]] = []
    for z in zones:
        intervals = _merge_intervals(per_zone[z["id"]])
        if len(intervals) > 1: raise ManualCorridor(f"航线多次进出协调区 {z['code']}，无法自动排序")
        if intervals: spans.append((intervals[0][0], intervals[0][1], z))
    if not spans: raise ManualCorridor("航线不经过任何已定义协调区")
    spans.sort(key=lambda s: s[0])
    # 覆盖连续性 / 区间重叠判定
    cursor = 0.0
    for t0, t1, z in spans:
        if t0 > cursor + 1e-7: raise ManualCorridor(f"航线在 {cursor:.3f}~{t0:.3f} 段不在任何协调区内")
        if t0 < cursor - 1e-7: raise ManualCorridor(f"协调区 {z['code']} 与相邻区在航线上重叠，无法判定归属")
        cursor = max(cursor, t1)
    if cursor < 1.0 - 1e-7: raise ManualCorridor("航线末端不在任何协调区内")
    segments: list[dict[str, Any]] = []
    for seq, (t0, t1, z) in enumerate(spans, start=1):
        segments.append({"zone_id": z["id"], "zone_code": z["code"], "seq": seq, "t0": t0, "t1": t1,
                         "route": _slice_points(route, t0, t1)})
    return segments


def partition_by_declared_zones(route: list[list[float]], zones: list[sqlite3.Row]) -> list[dict[str, Any]]:
    """人工判定路径：值班员指定途经区序列，系统按航线参数均分成段，不再要求几何相交。"""
    segments: list[dict[str, Any]] = []
    n = len(zones)
    for seq, z in enumerate(zones, start=1):
        t0, t1 = (seq - 1) / n, seq / n
        segments.append({"zone_id": z["id"], "zone_code": z["code"], "seq": seq, "t0": t0, "t1": t1,
                         "route": _slice_points(route, t0, t1)})
    return segments


class Repository:
    def __init__(self, path: str | Path):
        self.conn = sqlite3.connect(str(path), check_same_thread=False, isolation_level=None)
        self._tx_lock = threading.Lock()
        self.conn.row_factory = sqlite3.Row; self.conn.execute("PRAGMA foreign_keys=ON")
        self.conn.execute("PRAGMA journal_mode=WAL"); self.conn.execute("PRAGMA busy_timeout=5000")
        self.conn.executescript("""
        CREATE TABLE IF NOT EXISTS zones(
            id INTEGER PRIMARY KEY AUTOINCREMENT, code TEXT NOT NULL UNIQUE, name TEXT NOT NULL, seq INTEGER NOT NULL UNIQUE,
            min_lon REAL NOT NULL, min_lat REAL NOT NULL, max_lon REAL NOT NULL, max_lat REAL NOT NULL,
            capacity INTEGER NOT NULL, status TEXT NOT NULL DEFAULT 'active', created_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS restrictions(
            id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, kind TEXT NOT NULL, zone_id INTEGER REFERENCES zones(id),
            min_lon REAL NOT NULL, min_lat REAL NOT NULL, max_lon REAL NOT NULL, max_lat REAL NOT NULL,
            min_altitude REAL NOT NULL DEFAULT 0, max_altitude REAL NOT NULL,
            starts_at TEXT NOT NULL, ends_at TEXT NOT NULL, reason TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'active', created_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS flight_plans(
            id INTEGER PRIMARY KEY AUTOINCREMENT, operator_id TEXT NOT NULL, callsign TEXT NOT NULL, drone_model TEXT NOT NULL,
            payload_kg REAL NOT NULL, route_json TEXT NOT NULL, starts_at TEXT NOT NULL, ends_at TEXT NOT NULL, max_altitude REAL NOT NULL,
            population_risk INTEGER NOT NULL, emergency_plan TEXT NOT NULL, region TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'draft',
            revision INTEGER NOT NULL DEFAULT 1, created_by TEXT NOT NULL, created_at TEXT NOT NULL, updated_at TEXT NOT NULL,
            UNIQUE(operator_id,callsign,starts_at)
        );
        CREATE TABLE IF NOT EXISTS plan_segments(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), plan_revision INTEGER NOT NULL,
            zone_id INTEGER NOT NULL REFERENCES zones(id), seq INTEGER NOT NULL, t0 REAL NOT NULL, t1 REAL NOT NULL,
            route_json TEXT NOT NULL, enters_at TEXT NOT NULL, exits_at TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'pending',
            accepted_by TEXT, accepted_at TEXT, accept_offline_id TEXT, basis_json TEXT, voucher_id TEXT,
            released_by TEXT, released_at TEXT, release_offline_id TEXT, updated_at TEXT NOT NULL,
            UNIQUE(plan_id,plan_revision,seq),
            UNIQUE(zone_id,accept_offline_id),
            UNIQUE(zone_id,release_offline_id)
        );
        CREATE TABLE IF NOT EXISTS manual_reviews(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), plan_revision INTEGER NOT NULL,
            reason TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'open', created_by TEXT NOT NULL, created_at TEXT NOT NULL,
            resolved_at TEXT, resolution_json TEXT
        );
        CREATE TABLE IF NOT EXISTS approvals(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), plan_revision INTEGER NOT NULL,
            reviewer TEXT NOT NULL, decision TEXT NOT NULL, reason TEXT NOT NULL, offline_id TEXT UNIQUE,
            override_kind TEXT, created_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS notifications(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), kind TEXT NOT NULL,
            message TEXT NOT NULL, dedupe_key TEXT UNIQUE, created_at TEXT NOT NULL, delivered INTEGER NOT NULL DEFAULT 0
        );
        CREATE TABLE IF NOT EXISTS audit_log(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER, actor TEXT NOT NULL, role TEXT NOT NULL, action TEXT NOT NULL,
            detail_json TEXT NOT NULL, created_at TEXT NOT NULL
        );
        """)

    @contextmanager
    def tx(self):
        # 单连接 + 工作线程：全局锁串行化事务，保证并发交接只有一个先写入者生效
        with self._tx_lock:
            self.conn.execute("BEGIN IMMEDIATE")
            try: yield self.conn; self.conn.execute("COMMIT")
            except Exception: self.conn.execute("ROLLBACK"); raise

    @staticmethod
    def audit(conn: sqlite3.Connection, plan_id: int | None, actor: str, role: str, action: str, detail: dict[str, Any]) -> None:
        conn.execute("INSERT INTO audit_log(plan_id,actor,role,action,detail_json,created_at) VALUES(?,?,?,?,?,?)",
                     (plan_id, actor, role, action, json.dumps(detail, ensure_ascii=False, sort_keys=True), iso()))

    @staticmethod
    def notify(conn: sqlite3.Connection, plan_id: int, kind: str, message: str, dedupe_key: str | None = None) -> bool:
        # dedupe_key 使服务重启 / 重放时同一事件只产生一条通知
        cur = conn.execute("INSERT INTO notifications(plan_id,kind,message,dedupe_key,created_at) VALUES(?,?,?,?,?)",
                           (plan_id, kind, message, dedupe_key, iso())) if dedupe_key else \
              conn.execute("INSERT INTO notifications(plan_id,kind,message,created_at) VALUES(?,?,?,?)", (plan_id, kind, message, iso()))
        return cur.rowcount > 0


class DroneAirspaceService:
    def __init__(self, path: str | Path): self.repo = Repository(path)

    @staticmethod
    def identity(headers: Any) -> tuple[str, str, str]:
        actor, role, operator = headers.get("X-User-Id", "").strip(), headers.get("X-Role", "").strip(), headers.get("X-Operator", "").strip()
        if not actor or role not in ROLES: raise ApiError(401, "unauthorized", "需要 X-User-Id 和有效 X-Role")
        if role == "operator" and not operator: raise ApiError(401, "operator_required", "运营方角色必须提供 X-Operator")
        return actor, role, operator

    @staticmethod
    def zone_header(headers: Any) -> str | None:
        code = headers.get("X-Zone-Code", "").strip()
        return code or None

    @staticmethod
    def _dict(row: sqlite3.Row | None) -> dict[str, Any] | None: return dict(row) if row else None

    # ---------------------------------------------------------------- 协调区

    def create_zone(self, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "zone_forbidden", "只有空域审核员或指挥官可以维护协调区")
        code, name = str(body.get("code", "")).strip(), str(body.get("name", "")).strip()
        if not code or not name: raise ApiError(400, "invalid_zone", "协调区代码和名称必填")
        try:
            seq = int(body["seq"]); capacity = int(body.get("capacity", 1))
            min_lon, min_lat, max_lon, max_lat = map(float, (body.get("min_lon"), body.get("min_lat"), body.get("max_lon"), body.get("max_lat")))
        except (KeyError, TypeError, ValueError): raise ApiError(400, "invalid_zone", "序号、容量和空域范围必须为数字")
        if seq < 1 or capacity < 1 or min_lon >= max_lon or min_lat >= max_lat: raise ApiError(400, "invalid_zone", "序号、容量或空域范围无效")
        with self.repo.tx() as conn:
            try:
                cur = conn.execute("""INSERT INTO zones(code,name,seq,min_lon,min_lat,max_lon,max_lat,capacity,created_at)
                                      VALUES(?,?,?,?,?,?,?,?,?)""", (code, name, seq, min_lon, min_lat, max_lon, max_lat, capacity, iso()))
            except sqlite3.IntegrityError as exc: raise ApiError(409, "zone_duplicate", "协调区代码或序号已存在") from exc
            row = conn.execute("SELECT * FROM zones WHERE id=?", (cur.lastrowid,)).fetchone()
            Repository.audit(conn, None, actor, role, "zone_created", {"code": code, "seq": seq})
            return dict(row)

    def list_zones(self) -> dict[str, Any]:
        return {"zones": [dict(r) for r in self.repo.conn.execute("SELECT * FROM zones WHERE status='active' ORDER BY seq")]}

    # ---------------------------------------------------------------- 限制

    def create_restriction(self, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "restriction_forbidden", "只有空域审核员或指挥官可以维护限制")
        name, kind, reason = str(body.get("name", "")).strip(), str(body.get("kind", "")).strip(), str(body.get("reason", "")).strip()
        if kind not in {"no_fly", "temporary_limit"} or not name or not reason: raise ApiError(400, "invalid_restriction", "名称、类型和原因必填")
        try:
            min_lon, min_lat, max_lon, max_lat = map(float, (body.get("min_lon"), body.get("min_lat"), body.get("max_lon"), body.get("max_lat")))
            min_alt, max_alt = float(body.get("min_altitude", 0)), float(body.get("max_altitude"))
        except (TypeError, ValueError): raise ApiError(400, "invalid_restriction", "空域范围和高度必须为数字")
        start, end = parse_time(body.get("starts_at")), parse_time(body.get("ends_at"))
        if min_lon >= max_lon or min_lat >= max_lat or min_alt < 0 or max_alt <= min_alt or end <= start:
            raise ApiError(400, "invalid_restriction", "空域范围、高度或时间无效")
        with self.repo.tx() as conn:
            zone = conn.execute("SELECT * FROM zones WHERE status='active' AND min_lon<=? AND max_lon>=? AND min_lat<=? AND max_lat>=? LIMIT 1",
                                (max_lon, min_lon, max_lat, min_lat)).fetchone()
            cur = conn.execute("""INSERT INTO restrictions(name,kind,zone_id,min_lon,min_lat,max_lon,max_lat,min_altitude,max_altitude,starts_at,ends_at,reason,created_at)
                                  VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                               (name, kind, zone["id"] if zone else None, min_lon, min_lat, max_lon, max_lat, min_alt, max_alt, iso(start), iso(end), reason, iso()))
            restriction = conn.execute("SELECT * FROM restrictions WHERE id=?", (cur.lastrowid,)).fetchone()
            Repository.audit(conn, None, actor, role, "restriction_created", {"restriction_id": restriction["id"], "zone_id": restriction["zone_id"]})
            voided = self._void_pending_for_restriction(conn, restriction)
            return {**dict(restriction), "voided_segments": voided}

    def _void_pending_for_restriction(self, conn: sqlite3.Connection, restriction: sqlite3.Row) -> list[dict[str, Any]]:
        """限制变化：命中区内及下游所有「未确认」凭证作废，计划退回重排。已确认凭证保留（其接收快照可查）。"""
        rbox = (restriction["min_lon"], restriction["min_lat"], restriction["max_lon"], restriction["max_lat"])
        r_start, r_end = parse_time(restriction["starts_at"]), parse_time(restriction["ends_at"])
        outcomes: list[dict[str, Any]] = []
        plans = list(conn.execute("SELECT * FROM flight_plans WHERE status IN ('submitted','rescheduling')"))
        for plan in plans:
            segs = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND plan_revision=? ORDER BY seq", (plan["id"], plan["revision"])))
            if not segs: continue
            affected: int | None = None
            for seg in segs:
                if not boxes_overlap(route_bbox(json.loads(seg["route_json"])), rbox): continue
                if not times_overlap(parse_time(seg["enters_at"]), parse_time(seg["exits_at"]), r_start, r_end): continue
                if not (plan["max_altitude"] > restriction["min_altitude"] and restriction["max_altitude"] > 0): continue
                affected = seg["seq"]; break
            if affected is None: continue
            pending = [s for s in segs if s["seq"] >= affected and s["status"] == "pending"]
            for seg in pending:
                conn.execute("UPDATE plan_segments SET status='voided',updated_at=? WHERE id=?", (iso(), seg["id"]))
            if pending:
                conn.execute("UPDATE flight_plans SET status='rescheduling',updated_at=? WHERE id=?", (iso(), plan["id"]))
                seqs = [s["seq"] for s in pending]
                Repository.audit(conn, plan["id"], "system", "airspace_reviewer", "segments_voided",
                                 {"restriction_id": restriction["id"], "voided_seqs": seqs, "revision": plan["revision"]})
                Repository.notify(conn, plan["id"], "segments_voided",
                                  f"限制 {restriction['name']} 变化，第 {','.join(map(str, seqs))} 区段未确认凭证作废，计划退回重排",
                                  f"voided:{plan['id']}:{plan['revision']}:{restriction['id']}")
                outcomes.append({"plan_id": plan["id"], "revision": plan["revision"], "voided_seqs": seqs})
        return outcomes

    # ---------------------------------------------------------------- 计划

    def create_plan(self, actor: str, role: str, operator: str, body: dict[str, Any]) -> dict[str, Any]:
        if role != "operator": raise ApiError(403, "plan_forbidden", "只有运营方可以创建飞行计划")
        required = ("callsign", "drone_model", "starts_at", "ends_at", "emergency_plan", "region")
        if any(body.get(key) in (None, "") for key in required): raise ApiError(400, "missing_fields", "飞行计划字段不完整")
        route = validate_route(body.get("route")); start, end = parse_time(body["starts_at"]), parse_time(body["ends_at"])
        try: payload, altitude = float(body.get("payload_kg")), float(body.get("max_altitude"))
        except (TypeError, ValueError): raise ApiError(400, "invalid_numbers", "payload_kg 和 max_altitude 必须为数字")
        risk = body.get("population_risk")
        if not 0 <= payload <= 25 or altitude <= 0 or not isinstance(risk, int) or not 0 <= risk <= 5:
            raise ApiError(400, "invalid_plan", "载荷、高度或人口风险无效")
        if end <= start or start <= utcnow(): raise ApiError(400, "invalid_time", "飞行时间必须在未来且结束晚于开始")
        bbox = route_bbox(route)
        with self.repo.tx() as conn:
            try:
                cur = conn.execute("""INSERT INTO flight_plans(operator_id,callsign,drone_model,payload_kg,route_json,starts_at,ends_at,max_altitude,population_risk,emergency_plan,region,created_by,created_at,updated_at)
                                      VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                                   (operator, str(body["callsign"]).upper(), body["drone_model"], payload, json.dumps(route), iso(start), iso(end), altitude, risk, body["emergency_plan"], body["region"], actor, iso(), iso()))
            except sqlite3.IntegrityError as exc: raise ApiError(409, "plan_duplicate", "同一运营方、呼号和起飞时间的计划已存在") from exc
            plan_id = cur.lastrowid; Repository.audit(conn, plan_id, actor, role, "plan_created", {"bbox": bbox, "revision": 1})
            return self.get_plan(plan_id, role, operator)

    def _plan_row(self, conn: sqlite3.Connection, plan_id: int) -> sqlite3.Row:
        row = conn.execute("SELECT * FROM flight_plans WHERE id=?", (plan_id,)).fetchone()
        if not row: raise ApiError(404, "plan_not_found", "飞行计划不存在")
        return row

    @staticmethod
    def _route(row: sqlite3.Row) -> list[list[float]]: return json.loads(row["route_json"])

    def check_conflicts(self, plan_id: int, role: str, operator: str) -> dict[str, Any]:
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if role == "operator" and plan["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能查看其他运营方计划")
            if role not in {"operator", "airspace_reviewer", "commander", "auditor", "viewer"}: raise ApiError(403, "check_forbidden", "无权检查冲突")
            return self._conflict_report(conn, plan)

    def _conflict_report(self, conn: sqlite3.Connection, plan: sqlite3.Row) -> dict[str, Any]:
        route = self._route(plan); bbox = route_bbox(route); start, end = parse_time(plan["starts_at"]), parse_time(plan["ends_at"])
        hard: list[dict[str, Any]] = []; blocking: list[dict[str, Any]] = []
        if plan["payload_kg"] > 25: hard.append({"code": "payload_limit", "message": "载荷超过 25kg 硬限制"})
        if plan["max_altitude"] > 120: hard.append({"code": "altitude_limit", "message": "常规计划高度不得超过 120m"})
        if plan["population_risk"] > 3: blocking.append({"code": "population_risk", "risk": plan["population_risk"], "message": "人口风险超过常规批准阈值"})
        for restriction in conn.execute("SELECT * FROM restrictions WHERE status='active'"):
            rbox = (restriction["min_lon"], restriction["min_lat"], restriction["max_lon"], restriction["max_lat"])
            if not boxes_overlap(bbox, rbox): continue
            if not times_overlap(start, end, parse_time(restriction["starts_at"]), parse_time(restriction["ends_at"])): continue
            altitude_overlap = plan["max_altitude"] > restriction["min_altitude"] and restriction["max_altitude"] > 0
            if altitude_overlap:
                item = {"code": "airspace_restriction", "restriction_id": restriction["id"], "name": restriction["name"], "kind": restriction["kind"], "reason": restriction["reason"]}
                blocking.append(item)
        adjacent: list[dict[str, Any]] = []
        for other in conn.execute("SELECT * FROM flight_plans WHERE id!=? AND status IN ('submitted','approved') AND starts_at<? AND ends_at>?", (plan["id"], iso(end), iso(start))):
            if boxes_overlap(bbox, route_bbox(self._route(other)), 0.002):
                adjacent.append({"plan_id": other["id"], "callsign": other["callsign"], "operator_id": other["operator_id"], "status": other["status"], "starts_at": other["starts_at"], "ends_at": other["ends_at"]})
        if adjacent: blocking.append({"code": "adjacent_traffic", "plans": adjacent, "message": "相邻航路与有效计划重叠"})
        return {"plan_id": plan["id"], "revision": plan["revision"], "hard_violations": hard, "blocking_conflicts": blocking, "approvable": not hard and not blocking}

    def _segment_conflicts(self, conn: sqlite3.Connection, plan: sqlite3.Row, seg: sqlite3.Row) -> list[dict[str, Any]]:
        """该区段接收时刻的限制快照依据（按当时限制）。"""
        bbox = route_bbox(json.loads(seg["route_json"]))
        s_start, s_end = parse_time(seg["enters_at"]), parse_time(seg["exits_at"])
        hits: list[dict[str, Any]] = []
        for r in conn.execute("SELECT * FROM restrictions WHERE status='active'"):
            if not boxes_overlap(bbox, (r["min_lon"], r["min_lat"], r["max_lon"], r["max_lat"])): continue
            if not times_overlap(s_start, s_end, parse_time(r["starts_at"]), parse_time(r["ends_at"])): continue
            if plan["max_altitude"] > r["min_altitude"] and r["max_altitude"] > 0:
                hits.append({"restriction_id": r["id"], "name": r["name"], "kind": r["kind"], "reason": r["reason"]})
        return hits

    def _capacity_occupants(self, conn: sqlite3.Connection, seg: sqlite3.Row) -> list[dict[str, Any]]:
        return [dict(r) for r in conn.execute("""
            SELECT s.plan_id, p.callsign, s.seq, s.enters_at, s.exits_at
            FROM plan_segments s JOIN flight_plans p ON p.id=s.plan_id
            WHERE s.zone_id=? AND s.status='accepted' AND s.plan_id!=?
              AND p.revision=s.plan_revision AND p.status='submitted'
              AND s.enters_at < ? AND s.exits_at > ?""",
            (seg["zone_id"], seg["plan_id"], seg["exits_at"], seg["enters_at"]))]

    def get_plan(self, plan_id: int, role: str, operator: str = "") -> dict[str, Any]:
        conn = self.repo.conn; row = self._plan_row(conn, plan_id)
        if role == "operator" and row["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能查看其他运营方计划")
        result = dict(row); result["route"] = json.loads(result.pop("route_json")); result["route_bbox"] = route_bbox(result["route"])
        if role == "viewer":
            result = {key: result[key] for key in ("id", "callsign", "starts_at", "ends_at", "max_altitude", "region", "status", "valid_until" if "valid_until" in result else "updated_at")}
        if role in {"airspace_reviewer", "commander", "auditor"}: result["approvals"] = [dict(r) for r in conn.execute("SELECT * FROM approvals WHERE plan_id=? ORDER BY id", (plan_id,))]
        if role in {"operator", "airspace_reviewer", "commander", "auditor"}:
            result["segments"] = self._segments_view(conn, row)
            result["corridor"] = self._corridor_summary(conn, row)
            manual = conn.execute("SELECT * FROM manual_reviews WHERE plan_id=? ORDER BY id DESC LIMIT 1", (plan_id,)).fetchone()
            if manual: result["manual_review"] = dict(manual)
        return result

    def _segments_view(self, conn: sqlite3.Connection, plan: sqlite3.Row) -> list[dict[str, Any]]:
        out = []
        for seg in conn.execute("""SELECT s.*, z.code AS zone_code, z.name AS zone_name, z.capacity AS zone_capacity
                                  FROM plan_segments s JOIN zones z ON z.id=s.zone_id
                                  WHERE s.plan_id=? AND s.plan_revision=? ORDER BY s.seq""", (plan["id"], plan["revision"])):
            item = {k: seg[k] for k in seg.keys() if k != "route_json"}; item["route"] = json.loads(seg["route_json"])
            item["basis"] = json.loads(seg["basis_json"]) if seg["basis_json"] else None
            if seg["status"] in {"accepted", "released", "superseded"}:
                current_ids = sorted(r["restriction_id"] for r in self._segment_conflicts(conn, plan, seg))
                snapshot_ids = sorted(r["restriction_id"] for r in (item["basis"] or {}).get("restrictions", []))
                item["basis_current"] = current_ids == snapshot_ids
            out.append(item)
        return out

    def _corridor_summary(self, conn: sqlite3.Connection, plan: sqlite3.Row) -> dict[str, Any]:
        segs = list(conn.execute("""SELECT s.*, z.code AS zone_code FROM plan_segments s JOIN zones z ON z.id=s.zone_id
                                    WHERE s.plan_id=? AND s.plan_revision=? ORDER BY s.seq""", (plan["id"], plan["revision"])))
        manual = conn.execute("SELECT status FROM manual_reviews WHERE plan_id=? AND plan_revision=? ORDER BY id DESC LIMIT 1",
                              (plan["id"], plan["revision"])).fetchone()
        if not segs:
            return {"status": "manual" if manual and manual["status"] == "open" else "none", "current_zone": None, "gate_seq": None}
        released = [s for s in segs if s["status"] == "released"]
        pending_or_active = [s for s in segs if s["status"] in {"pending", "accepted"}]
        gate = min((s["seq"] for s in pending_or_active), default=None)
        if gate is None:
            voided = next((s for s in segs if s["status"] == "voided"), None)
            if voided:
                return {"status": "voided", "current_zone": voided["zone_code"], "gate_seq": voided["seq"],
                        "released_seqs": [s["seq"] for s in released]}
            current = segs[-1]; state = "completed"
        else:
            current = next(s for s in segs if s["seq"] == gate)
            stale = False
            if current["status"] == "accepted":
                basis = json.loads(current["basis_json"]) if current["basis_json"] else {}
                now_ids = sorted(r["restriction_id"] for r in self._segment_conflicts(conn, plan, current))
                stale = now_ids != sorted(r["restriction_id"] for r in basis.get("restrictions", []))
            state = "voided" if current["status"] == "voided" or stale else "flowing"
        return {"status": state, "current_zone": current["zone_code"], "gate_seq": gate,
                "released_seqs": [s["seq"] for s in released]}

    # ------------------------------------------------------------ 区段构建

    def _build_segments(self, conn: sqlite3.Connection, plan: sqlite3.Row, explicit_codes: list[str] | None = None) -> list[dict[str, Any]]:
        zones = list(conn.execute("SELECT * FROM zones WHERE status='active' ORDER BY seq"))
        declared = False
        if explicit_codes:
            by_code = {z["code"]: z for z in zones}
            zones = []
            for code in explicit_codes:
                if code not in by_code: raise ApiError(400, "zone_unknown", f"协调区 {code} 不存在")
                zones.append(by_code[code])
            declared = True
        route = self._route(plan); start, end = parse_time(plan["starts_at"]), parse_time(plan["ends_at"])
        cut = partition_by_declared_zones(route, zones) if declared else split_route_by_zones(route, zones)
        for seg in cut:
            seg["enters_at"] = iso(start + (end - start) * seg["t0"])
            seg["exits_at"] = iso(start + (end - start) * seg["t1"])
        return cut

    def _persist_segments(self, conn: sqlite3.Connection, plan: sqlite3.Row, cut: list[dict[str, Any]], actor: str, role: str, source: str) -> None:
        for seg in cut:
            conn.execute("""INSERT INTO plan_segments(plan_id,plan_revision,zone_id,seq,t0,t1,route_json,enters_at,exits_at,status,updated_at)
                            VALUES(?,?,?,?,?,?,?,?,?, 'pending',?)""",
                         (plan["id"], plan["revision"], seg["zone_id"], seg["seq"], seg["t0"], seg["t1"],
                          json.dumps(seg["route"], ensure_ascii=False), seg["enters_at"], seg["exits_at"], iso()))
        Repository.audit(conn, plan["id"], actor, role, "segments_built",
                         {"revision": plan["revision"], "seq": [s["seq"] for s in cut], "source": source})

    def backfill_segments(self, plan_id: int, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        """旧计划回填：按航线与协调区（含限制所在范围）自动切区；无法唯一判定则入人工队列。"""
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "backfill_forbidden", "只有空域审核员或指挥官可以回填区段")
        explicit = body.get("zones")
        if explicit is not None and (not isinstance(explicit, list) or not all(isinstance(c, str) for c in explicit)):
            raise ApiError(400, "invalid_zones", "zones 必须是协调区代码数组")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if plan["status"] in {"canceled", "expired"}: raise ApiError(409, "plan_closed", "已取消或过期计划不能回填")
            existing = conn.execute("SELECT COUNT(*) AS c FROM plan_segments WHERE plan_id=? AND plan_revision=?", (plan_id, plan["revision"])).fetchone()["c"]
            open_manual = conn.execute("SELECT * FROM manual_reviews WHERE plan_id=? AND plan_revision=? AND status='open' ORDER BY id DESC LIMIT 1",
                                       (plan_id, plan["revision"])).fetchone()
            if existing and not open_manual:
                return {"plan_id": plan_id, "segments": self._segments_view(conn, plan), "idempotent": True}
            try: cut = self._build_segments(conn, plan, explicit)  # type: ignore[arg-type]
            except ManualCorridor as exc:
                if not open_manual:
                    conn.execute("INSERT INTO manual_reviews(plan_id,plan_revision,reason,created_by,created_at) VALUES(?,?,?,?,?)",
                                 (plan_id, plan["revision"], str(exc), actor, iso()))
                    Repository.notify(conn, plan_id, "corridor_manual", f"计划无法自动切区，交人工处理：{exc}", f"manual:{plan_id}:{plan['revision']}")
                Repository.audit(conn, plan_id, actor, role, "manual_queued", {"reason": str(exc)})
                return {"plan_id": plan_id, "manual": True, "reason": str(exc)}
            if open_manual:
                conn.execute("UPDATE manual_reviews SET status='resolved', resolved_at=?, resolution_json=? WHERE id=?",
                             (iso(), json.dumps({"by": actor, "zones": [s["zone_code"] for s in cut]}, ensure_ascii=False), open_manual["id"]))
            if not existing: self._persist_segments(conn, plan, cut, actor, role, "backfill")
            return {"plan_id": plan_id, "manual": False, "segments": self._segments_view(conn, plan)}

    # ------------------------------------------------------------ 提交/审核

    def submit(self, plan_id: int, actor: str, role: str, operator: str, body: dict[str, Any]) -> dict[str, Any]:
        if role != "operator": raise ApiError(403, "submit_forbidden", "只有运营方可以提交计划")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if plan["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能提交其他运营方计划")
            if plan["status"] == "submitted": return {"plan": self.get_plan(plan_id, role, operator), "idempotent": True}
            if plan["status"] not in {"draft", "rejected", "rescheduling"}: raise ApiError(409, "invalid_transition", "当前状态不能提交")
            if parse_time(plan["starts_at"]) <= utcnow(): raise ApiError(409, "plan_expired", "计划起飞时间已过")
            conn.execute("UPDATE flight_plans SET status='submitted',updated_at=? WHERE id=?", (iso(), plan_id))
            plan = self._plan_row(conn, plan_id)
            corridor: dict[str, Any] = {"manual": False}
            if not conn.execute("SELECT COUNT(*) AS c FROM plan_segments WHERE plan_id=? AND plan_revision=?", (plan_id, plan["revision"])).fetchone()["c"]:
                try:
                    cut = self._build_segments(conn, plan)
                    self._persist_segments(conn, plan, cut, actor, role, "submit")
                except ManualCorridor as exc:
                    conn.execute("INSERT INTO manual_reviews(plan_id,plan_revision,reason,created_by,created_at) VALUES(?,?,?,?,?)",
                                 (plan_id, plan["revision"], str(exc), actor, iso()))
                    Repository.notify(conn, plan_id, "corridor_manual", f"计划无法自动切区，交人工处理：{exc}", f"manual:{plan_id}:{plan['revision']}")
                    Repository.audit(conn, plan_id, actor, role, "manual_queued", {"reason": str(exc)})
                    corridor = {"manual": True, "reason": str(exc)}
            Repository.audit(conn, plan_id, actor, role, "plan_submitted", {"revision": plan["revision"]})
            return {"plan": self.get_plan(plan_id, role, operator), "idempotent": False, "corridor": corridor}

    def approve(self, plan_id: int, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "review_forbidden", "只有空域审核员或指挥官可以批准")
        expected, offline_id = body.get("expected_revision"), str(body.get("offline_id", "")).strip()
        reason, override = str(body.get("reason", "")).strip(), str(body.get("override_reason", "")).strip()
        if not isinstance(expected, int) or not offline_id or not reason: raise ApiError(400, "review_details_required", "expected_revision、offline_id 和 reason 必填")
        with self.repo.tx() as conn:
            prior = conn.execute("SELECT * FROM approvals WHERE offline_id=?", (offline_id,)).fetchone()
            if prior:
                if prior["plan_id"] == plan_id and prior["plan_revision"] == expected and prior["decision"] == "approved":
                    return {"plan": self.get_plan(plan_id, role, ""), "idempotent": True, "approval_id": prior["id"]}
                raise ApiError(409, "offline_id_conflict", "该离线审核编号已经用于其他决定")
            plan = self._plan_row(conn, plan_id)
            if plan["status"] == "approved": return {"plan": self.get_plan(plan_id, role, ""), "idempotent": True}
            if plan["status"] != "submitted": raise ApiError(409, "invalid_transition", "只有已提交计划可以批准")
            if plan["revision"] != expected: raise ApiError(409, "revision_conflict", "计划版本已变化，审核决定不能套用")
            report = self._conflict_report(conn, plan)
            if report["hard_violations"]: raise ApiError(409, "hard_constraint_violation", "计划违反不可覆盖的安全约束", report)
            if report["blocking_conflicts"] and not (role == "commander" and override):
                raise ApiError(409, "airspace_conflict", "计划存在空域或相邻交通冲突", report)
            override_kind = "emergency_authority" if report["blocking_conflicts"] else None
            cur = conn.execute("""INSERT INTO approvals(plan_id,plan_revision,reviewer,decision,reason,offline_id,override_kind,created_at)
                                  VALUES(?,?,?,?,?,?,?,?)""", (plan_id, expected, actor, "approved", reason, offline_id, override_kind, iso()))
            conn.execute("UPDATE flight_plans SET status='approved',updated_at=? WHERE id=?", (iso(), plan_id))
            if override_kind: Repository.audit(conn, plan_id, actor, role, "emergency_override_used", {"override_reason": override, "conflicts": report["blocking_conflicts"]})
            Repository.audit(conn, plan_id, actor, role, "plan_approved", {"revision": expected, "offline_id": offline_id})
            Repository.notify(conn, plan_id, "approved", f"飞行计划 {plan['callsign']} 已批准", f"legacy-approved:{plan_id}:{expected}")
            return {"plan": self.get_plan(plan_id, role, ""), "idempotent": False, "approval_id": cur.lastrowid, "override_kind": override_kind}

    def reject(self, plan_id: int, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "review_forbidden", "当前角色不能拒绝计划")
        expected, offline_id, reason = body.get("expected_revision"), str(body.get("offline_id", "")).strip(), str(body.get("reason", "")).strip()
        if not isinstance(expected, int) or not offline_id or not reason: raise ApiError(400, "review_details_required", "expected_revision、offline_id 和 reason 必填")
        with self.repo.tx() as conn:
            prior = conn.execute("SELECT * FROM approvals WHERE offline_id=?", (offline_id,)).fetchone()
            if prior:
                if prior["plan_id"] == plan_id and prior["plan_revision"] == expected and prior["decision"] == "rejected": return {"plan": self.get_plan(plan_id, role, ""), "idempotent": True}
                raise ApiError(409, "offline_id_conflict", "该离线审核编号已经被使用")
            plan = self._plan_row(conn, plan_id)
            if plan["status"] != "submitted" or plan["revision"] != expected: raise ApiError(409, "revision_conflict", "计划状态或版本不匹配")
            conn.execute("INSERT INTO approvals(plan_id,plan_revision,reviewer,decision,reason,offline_id,created_at) VALUES(?,?,?,?,?,?,?)", (plan_id, expected, actor, "rejected", reason, offline_id, iso()))
            conn.execute("UPDATE plan_segments SET status='voided',updated_at=? WHERE plan_id=? AND plan_revision=? AND status='pending'", (iso(), plan_id, expected))
            conn.execute("UPDATE flight_plans SET status='rejected',updated_at=? WHERE id=?", (iso(), plan_id))
            Repository.audit(conn, plan_id, actor, role, "plan_rejected", {"reason": reason, "offline_id": offline_id})
            Repository.notify(conn, plan_id, "rejected", f"飞行计划 {plan['callsign']} 被拒绝：{reason}", f"rejected:{plan_id}:{expected}:{offline_id}")
            return {"plan": self.get_plan(plan_id, role, ""), "idempotent": False}

    # -------------------------------------------------------- 逐区接收/放行

    def _zone_row(self, conn: sqlite3.Connection, code: str | None) -> sqlite3.Row | None:
        if not code: return None
        row = conn.execute("SELECT * FROM zones WHERE code=? AND status='active'", (code,)).fetchone()
        if not row: raise ApiError(400, "zone_unknown", f"协调区 {code} 不存在")
        return row

    def _guard_segment_action(self, conn: sqlite3.Connection, plan_id: int, role: str, zone_code: str | None):
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "zone_action_forbidden", "只有值班审核员或指挥官可以操作区段")
        plan = self._plan_row(conn, plan_id)
        if plan["status"] not in {"submitted", "rescheduling"}: raise ApiError(409, "invalid_transition", "计划当前状态不能进行区段交接")
        manual = conn.execute("SELECT status FROM manual_reviews WHERE plan_id=? AND plan_revision=? AND status='open' ORDER BY id DESC LIMIT 1",
                              (plan_id, plan["revision"])).fetchone()
        if manual: raise ApiError(409, "corridor_manual", "计划区段无法自动判定，等待人工处理")
        return plan

    def accept_segment(self, plan_id: int, seq: int, actor: str, role: str, zone_code: str | None, body: dict[str, Any]) -> dict[str, Any]:
        offline_id = str(body.get("offline_id", "")).strip()
        if not offline_id: raise ApiError(400, "offline_required", "区段接收必须携带按区编号的 offline_id")
        expected = body.get("expected_revision")
        if expected is not None and not isinstance(expected, int): raise ApiError(400, "invalid_revision", "expected_revision 必须是整数")
        note = str(body.get("note", "")).strip()
        with self.repo.tx() as conn:
            plan = self._guard_segment_action(conn, plan_id, role, zone_code)
            if expected is not None and expected != plan["revision"]: raise ApiError(409, "revision_conflict", "计划版本已变化，接收凭证不能套用")
            zone = self._zone_row(conn, zone_code)
            seg = conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND plan_revision=? AND seq=?", (plan_id, plan["revision"], seq)).fetchone()
            if not seg: raise ApiError(404, "segment_not_found", "当前版本没有该区段")
            seg_zone = conn.execute("SELECT * FROM zones WHERE id=?", (seg["zone_id"],)).fetchone()
            if role == "airspace_reviewer" and (not zone or zone["id"] != seg["zone_id"]):
                raise ApiError(403, "zone_mismatch", "值班员只能确认本协调区", {"required_zone": seg_zone["code"]})
            # 断线回传按区编号合并：offline_id 只在本区内唯一，跨区重名互不冲突
            prior = conn.execute("SELECT * FROM plan_segments WHERE accept_offline_id=? AND zone_id=?",
                                 (offline_id, seg_zone["id"])).fetchone()
            if prior:
                if prior["plan_id"] == plan_id and prior["plan_revision"] == plan["revision"] and prior["seq"] == seq:
                    return {"plan_id": plan_id, "seq": seq, "voucher_id": prior["voucher_id"], "status": prior["status"], "idempotent": True}
                raise ApiError(409, "offline_zone_conflict", "该区下的离线编号已用于其他计划或区段",
                               {"plan_id": prior["plan_id"], "seq": prior["seq"]})
            # 顺序闸门：上一区放行后，下一区才能确认
            summary = self._corridor_summary(conn, plan)
            if seg["status"] == "voided":
                raise ApiError(409, "segment_voided", "该区段凭证已作废，计划需退回重排", summary)
            if seg["status"] in {"accepted", "released", "superseded"}:
                raise ApiError(409, "handoff_conflict", "该区段已有生效交接，先写入的接收生效",
                               {**summary, "held_by": seg["accepted_by"], "voucher_id": seg["voucher_id"]})
            if summary["gate_seq"] != seq:
                raise ApiError(409, "out_of_order", "上一区尚未放行，不能提前确认本区",
                               {**summary, "requested_seq": seq})
            # 按当时限制确认
            conflicts = self._segment_conflicts(conn, plan, seg)
            if conflicts: raise ApiError(409, "zone_restriction", "区段与当前生效限制冲突", {"seq": seq, "zone": seg_zone["code"], "conflicts": conflicts})
            occupants = self._capacity_occupants(conn, seg)
            if len(occupants) >= seg_zone["capacity"]:
                raise ApiError(409, "capacity_full", f"协调区 {seg_zone['code']} 容量已满",
                               {"zone": seg_zone["code"], "capacity": seg_zone["capacity"], "occupants": occupants})
            voucher = f"V-P{plan_id}-R{plan['revision']}-S{seq}-{seg_zone['code']}"
            basis = {"restrictions": conflicts, "capacity": seg_zone["capacity"], "occupants": len(occupants), "accepted_at": iso(), "note": note}
            try:
                cur = conn.execute("""UPDATE plan_segments SET status='accepted', accepted_by=?, accepted_at=?, accept_offline_id=?,
                                basis_json=?, voucher_id=?, updated_at=? WHERE id=? AND status='pending'""",
                             (actor, iso(), offline_id, json.dumps(basis, ensure_ascii=False), voucher, iso(), seg["id"]))
            except sqlite3.IntegrityError as exc:
                winner_row = conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND plan_revision=? AND seq=?", (plan_id, plan["revision"], seq)).fetchone()
                raise ApiError(409, "handoff_conflict", "该区段已有生效交接，先写入的接收生效",
                               {**self._corridor_summary(conn, plan), "held_by": winner_row["accepted_by"], "voucher_id": winner_row["voucher_id"]}) from exc
            if cur.rowcount == 0:
                # 两个值班员同时确认同一槽位：先写入者赢，后到者看到当前所在区与冲突
                winner_row = conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND plan_revision=? AND seq=?", (plan_id, plan["revision"], seq)).fetchone()
                raise ApiError(409, "handoff_conflict", "该区段已有生效交接，先写入的接收生效",
                               {**self._corridor_summary(conn, plan), "held_by": winner_row["accepted_by"], "voucher_id": winner_row["voucher_id"]})
            Repository.audit(conn, plan_id, actor, role, "segment_accepted",
                             {"seq": seq, "zone": seg_zone["code"], "revision": plan["revision"], "offline_id": offline_id, "voucher_id": voucher})
            Repository.notify(conn, plan_id, "segment_accepted",
                              f"第 {seq} 区 {seg_zone['code']} 已接收计划 {plan['callsign']}（凭证 {voucher}）",
                              f"seg-accepted:{plan_id}:{plan['revision']}:{seq}")
            return {"plan_id": plan_id, "seq": seq, "zone": seg_zone["code"], "voucher_id": voucher,
                    "bound_revision": plan["revision"], "basis": basis, "idempotent": False}

    def release_segment(self, plan_id: int, seq: int, actor: str, role: str, zone_code: str | None, body: dict[str, Any]) -> dict[str, Any]:
        offline_id = str(body.get("offline_id", "")).strip()
        with self.repo.tx() as conn:
            plan = self._guard_segment_action(conn, plan_id, role, zone_code)
            seg = conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND plan_revision=? AND seq=?", (plan_id, plan["revision"], seq)).fetchone()
            if not seg: raise ApiError(404, "segment_not_found", "当前版本没有该区段")
            seg_zone = conn.execute("SELECT * FROM zones WHERE id=?", (seg["zone_id"],)).fetchone()
            zone = self._zone_row(conn, zone_code)
            if role == "airspace_reviewer" and (not zone or zone["id"] != seg["zone_id"]):
                raise ApiError(403, "zone_mismatch", "值班员只能放行本协调区", {"required_zone": seg_zone["code"]})
            if offline_id:
                prior = conn.execute("SELECT * FROM plan_segments WHERE release_offline_id=? AND zone_id=?",
                                     (offline_id, seg_zone["id"])).fetchone()
                if prior and prior["plan_id"] == plan_id and prior["plan_revision"] == plan["revision"]:
                    return {"plan_id": plan_id, "seq": seq, "status": prior["status"], "idempotent": True}
            if seg["status"] == "released":
                return {"plan_id": plan_id, "seq": seq, "status": "released", "idempotent": True}
            if seg["status"] != "accepted":
                raise ApiError(409, "not_accepted", "区段尚未接收，不能放行", self._corridor_summary(conn, plan))
            # 接收后限制已变化：凭证依据失效，不能放行，计划退回重排
            basis = json.loads(seg["basis_json"]) if seg["basis_json"] else {}
            current_hits = self._segment_conflicts(conn, plan, seg)
            if sorted(r["restriction_id"] for r in current_hits) != sorted(r["restriction_id"] for r in basis.get("restrictions", [])):
                raise ApiError(409, "zone_restriction_changed",
                               f"协调区 {seg_zone['code']} 限制在接收后发生变化，凭证依据失效，计划退回重排",
                               self._corridor_summary(conn, plan))
            if self._corridor_summary(conn, plan)["gate_seq"] != seq:
                raise ApiError(409, "out_of_order", "只能放行当前所在区", self._corridor_summary(conn, plan))
            conn.execute("UPDATE plan_segments SET status='released', released_by=?, released_at=?, release_offline_id=?, updated_at=? WHERE id=?",
                         (actor, iso(), offline_id or None, iso(), seg["id"]))
            Repository.audit(conn, plan_id, actor, role, "segment_released", {"seq": seq, "zone": seg_zone["code"], "revision": plan["revision"]})
            Repository.notify(conn, plan_id, "segment_released", f"第 {seq} 区 {seg_zone['code']} 已放行",
                              f"seg-released:{plan_id}:{plan['revision']}:{seq}")
            last_seq = conn.execute("SELECT MAX(seq) AS m FROM plan_segments WHERE plan_id=? AND plan_revision=?", (plan_id, plan["revision"])).fetchone()["m"]
            completed = seq == last_seq
            if completed:
                conn.execute("UPDATE flight_plans SET status='approved',updated_at=? WHERE id=?", (iso(), plan_id))
                Repository.notify(conn, plan_id, "approved", f"飞行计划 {plan['callsign']} 走廊全程放行",
                                  f"corridor-approved:{plan_id}:{plan['revision']}")
            else:
                nxt = conn.execute("SELECT z.code FROM plan_segments s JOIN zones z ON z.id=s.zone_id WHERE s.plan_id=? AND s.plan_revision=? AND s.seq=?",
                                   (plan_id, plan["revision"], seq + 1)).fetchone()
                Repository.notify(conn, plan_id, "handoff_ready", f"第 {seq + 1} 区 {nxt['code']} 可以确认接收",
                                  f"seg-ready:{plan_id}:{plan['revision']}:{seq + 1}")
            return {"plan_id": plan_id, "seq": seq, "zone": seg_zone["code"], "status": "released", "corridor_completed": completed}

    def sync_segments(self, plan_id: int, actor: str, role: str, zone_code: str | None, body: dict[str, Any]) -> dict[str, Any]:
        """断线回传批量合并：逐条按区编号幂等处理，只续办未完成区段。"""
        items = body.get("items")
        if not isinstance(items, list) or not items: raise ApiError(400, "items_required", "items 必须是非空数组")
        results = []
        for item in items:
            seq, action = item.get("seq"), item.get("action")
            if not isinstance(seq, int) or action not in {"accept", "release"}:
                results.append({"seq": seq, "ok": False, "error": "invalid_item", "message": "seq/action 无效"}); continue
            try:
                if action == "accept":
                    res = self.accept_segment(plan_id, seq, actor, role, zone_code, item)
                else:
                    res = self.release_segment(plan_id, seq, actor, role, zone_code, item)
                results.append({"seq": seq, "ok": True, **res})
            except ApiError as exc:
                results.append({"seq": seq, "ok": False, "error": exc.code, "message": exc.message, "details": exc.details})
        return {"plan_id": plan_id, "results": results, "finished": [r["seq"] for r in results if r["ok"]]}

    # ------------------------------------------------------------ 变更/取消

    def change(self, plan_id: int, actor: str, role: str, operator: str, body: dict[str, Any]) -> dict[str, Any]:
        if role != "operator": raise ApiError(403, "change_forbidden", "只有运营方可以变更计划")
        expected = body.get("expected_revision")
        if not isinstance(expected, int): raise ApiError(400, "revision_required", "expected_revision 必填")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if plan["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能修改其他运营方计划")
            if plan["status"] in {"canceled", "expired"}: raise ApiError(409, "plan_closed", "已取消或过期计划不能修改")
            if plan["revision"] != expected: raise ApiError(409, "revision_conflict", "计划版本已变化")
            route = validate_route(body.get("route", self._route(plan)))
            start = parse_time(body.get("starts_at", plan["starts_at"])); end = parse_time(body.get("ends_at", plan["ends_at"]))
            if end <= start or start <= utcnow(): raise ApiError(400, "invalid_time", "新飞行时间无效")
            payload = float(body.get("payload_kg", plan["payload_kg"])); altitude = float(body.get("max_altitude", plan["max_altitude"]))
            risk = body.get("population_risk", plan["population_risk"])
            if not 0 <= payload <= 25 or altitude <= 0 or not isinstance(risk, int) or not 0 <= risk <= 5: raise ApiError(400, "invalid_plan", "变更后的载荷、高度或风险无效")
            revision = expected + 1
            conn.execute("""UPDATE flight_plans SET route_json=?,starts_at=?,ends_at=?,payload_kg=?,max_altitude=?,population_risk=?,emergency_plan=?,region=?,status='draft',revision=?,updated_at=? WHERE id=?""",
                         (json.dumps(route), iso(start), iso(end), payload, altitude, risk, body.get("emergency_plan", plan["emergency_plan"]), body.get("region", plan["region"]), revision, iso(), plan_id))
            # 旧版本凭证随版本失效，释放容量；已放行区段保留为历史
            conn.execute("UPDATE plan_segments SET status='voided',updated_at=? WHERE plan_id=? AND plan_revision=? AND status='pending'", (iso(), plan_id, expected))
            conn.execute("UPDATE plan_segments SET status='superseded',updated_at=? WHERE plan_id=? AND plan_revision=? AND status='accepted'", (iso(), plan_id, expected))
            Repository.audit(conn, plan_id, actor, role, "plan_changed", {"from_revision": expected, "to_revision": revision, "previous_status": plan["status"]})
            if plan["status"] == "approved": Repository.notify(conn, plan_id, "approval_invalidated", f"飞行计划 {plan['callsign']} 已修改，原批准自动失效", f"invalidated:{plan_id}:{revision}")
            else: Repository.notify(conn, plan_id, "changed", f"飞行计划 {plan['callsign']} 已更新，需重新提交审核", f"changed:{plan_id}:{revision}")
            return self.get_plan(plan_id, role, operator)

    def cancel(self, plan_id: int, actor: str, role: str, operator: str, body: dict[str, Any]) -> dict[str, Any]:
        reason = str(body.get("reason", "")).strip()
        if not reason: raise ApiError(400, "reason_required", "取消原因必填")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if role == "operator" and plan["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能取消其他运营方计划")
            if role not in {"operator", "airspace_reviewer", "commander"}: raise ApiError(403, "cancel_forbidden", "当前角色不能取消计划")
            if plan["status"] == "canceled": return {"plan": self.get_plan(plan_id, role, operator), "idempotent": True}
            if plan["status"] == "expired": raise ApiError(409, "plan_expired", "已过期计划不能取消")
            conn.execute("UPDATE plan_segments SET status='voided',updated_at=? WHERE plan_id=? AND plan_revision=? AND status IN ('pending','accepted')",
                         (iso(), plan_id, plan["revision"]))
            conn.execute("UPDATE flight_plans SET status='canceled',updated_at=? WHERE id=?", (iso(), plan_id))
            Repository.audit(conn, plan_id, actor, role, "plan_canceled", {"reason": reason})
            Repository.notify(conn, plan_id, "canceled", f"飞行计划 {plan['callsign']} 已取消：{reason}", f"canceled:{plan_id}:{plan['revision']}")
            return {"plan": self.get_plan(plan_id, role, operator), "idempotent": False}

    def notifications(self, actor: str, role: str, operator: str) -> dict[str, Any]:
        if role == "operator":
            rows = self.repo.conn.execute("""SELECT n.* FROM notifications n JOIN flight_plans p ON p.id=n.plan_id WHERE p.operator_id=? ORDER BY n.id DESC""", (operator,))
        elif role in {"airspace_reviewer", "commander", "auditor"}: rows = self.repo.conn.execute("SELECT * FROM notifications ORDER BY id DESC")
        else: raise ApiError(403, "notifications_forbidden", "当前角色不能读取通知")
        return {"notifications": [dict(r) for r in rows]}

    def expire_plans(self, actor: str, role: str) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "expire_forbidden", "当前角色不能执行到期处理")
        now = iso()
        with self.repo.tx() as conn:
            rows = list(conn.execute("SELECT * FROM flight_plans WHERE status='approved' AND ends_at<=?", (now,)))
            for row in rows:
                conn.execute("UPDATE flight_plans SET status='expired',updated_at=? WHERE id=?", (now, row["id"]))
                Repository.audit(conn, row["id"], actor, role, "plan_expired", {})
                Repository.notify(conn, row["id"], "expired", f"飞行计划 {row['callsign']} 已过期", f"expired:{row['id']}:{row['revision']}")
        return {"expired": len(rows)}

    def state(self, role: str, operator: str) -> dict[str, Any]:
        conn = self.repo.conn
        if role == "operator": rows = conn.execute("SELECT * FROM flight_plans WHERE operator_id=? ORDER BY id DESC", (operator,))
        elif role in {"airspace_reviewer", "commander", "auditor"}: rows = conn.execute("SELECT * FROM flight_plans ORDER BY id DESC")
        else: rows = conn.execute("SELECT * FROM flight_plans WHERE status='approved' ORDER BY id DESC")
        plans = []
        for row in rows:
            item = self.get_plan(row["id"], role, operator); plans.append(item)
        is_staff = role in {"airspace_reviewer", "commander", "auditor"}
        restrictions = [dict(r) for r in conn.execute("SELECT * FROM restrictions WHERE status='active' ORDER BY id DESC")] if is_staff else []
        zones = [dict(r) for r in conn.execute("SELECT * FROM zones WHERE status='active' ORDER BY seq")] if role != "viewer" else []
        manual = [dict(r) for r in conn.execute("SELECT * FROM manual_reviews WHERE status='open' ORDER BY id DESC")] if is_staff else []
        return {"plans": plans, "restrictions": restrictions, "zones": zones, "manual_queue": manual, "server_time": iso()}


def send_json(handler: BaseHTTPRequestHandler, status: int, payload: Any) -> None:
    raw = json.dumps(payload, ensure_ascii=False, default=str).encode(); handler.send_response(status); handler.send_header("Content-Type", "application/json; charset=utf-8"); handler.send_header("Content-Length", str(len(raw))); handler.end_headers(); handler.wfile.write(raw)


class Handler(BaseHTTPRequestHandler):
    service: DroneAirspaceService; web_root: Path
    def log_message(self, fmt: str, *args: Any) -> None: print(f"{self.address_string()} - {fmt % args}")
    def body(self) -> dict[str, Any]:
        size = int(self.headers.get("Content-Length", "0"))
        if not size: return {}
        try: value = json.loads(self.rfile.read(size))
        except json.JSONDecodeError as exc: raise ApiError(400, "invalid_json", "请求体不是有效 JSON") from exc
        if not isinstance(value, dict): raise ApiError(400, "invalid_json", "请求体必须是对象")
        return value
    def get_api(self, path: str) -> tuple[int, Any]:
        if path == "/health": return 200, {"status": "ok", "service": "drone-airspace"}
        actor, role, operator = self.service.identity(self.headers)
        if path == "/api/zones": return 200, self.service.list_zones()
        if path == "/api/state": return 200, self.service.state(role, operator)
        if path == "/api/notifications": return 200, self.service.notifications(actor, role, operator)
        parts = [p for p in path.split("/") if p]
        if len(parts) == 3 and parts[:2] == ["api", "plans"] and parts[2].isdigit(): return 200, self.service.get_plan(int(parts[2]), role, operator)
        if len(parts) == 4 and parts[:2] == ["api", "plans"] and parts[2].isdigit() and parts[3] == "check": return 200, self.service.check_conflicts(int(parts[2]), role, operator)
        raise ApiError(404, "not_found", "接口不存在")
    def post_api(self, path: str) -> tuple[int, Any]:
        actor, role, operator = self.service.identity(self.headers); body = self.body()
        zone = self.service.zone_header(self.headers)
        parts = [p for p in path.split("/") if p]
        if path == "/api/zones": return 201, self.service.create_zone(actor, role, body)
        if path == "/api/restrictions": return 201, self.service.create_restriction(actor, role, body)
        if path == "/api/plans": return 201, self.service.create_plan(actor, role, operator, body)
        if path == "/api/expire": return 200, self.service.expire_plans(actor, role)
        if len(parts) == 4 and parts[:2] == ["api", "plans"] and parts[2].isdigit():
            pid, action = int(parts[2]), parts[3]
            routes = {
                "submit": lambda: self.service.submit(pid, actor, role, operator, body),
                "approve": lambda: self.service.approve(pid, actor, role, body),
                "reject": lambda: self.service.reject(pid, actor, role, body),
                "change": lambda: self.service.change(pid, actor, role, operator, body),
                "cancel": lambda: self.service.cancel(pid, actor, role, operator, body),
                "backfill": lambda: self.service.backfill_segments(pid, actor, role, body),
                "sync": lambda: self.service.sync_segments(pid, actor, role, zone, body),
            }
            if action in routes: return 200, routes[action]()
        if (len(parts) == 6 and parts[:2] == ["api", "plans"] and parts[2].isdigit()
                and parts[3] == "segments" and parts[4].isdigit() and parts[5] in {"accept", "release"}):
            pid, seq, action = int(parts[2]), int(parts[4]), parts[5]
            if action == "accept": return 200, self.service.accept_segment(pid, seq, actor, role, zone, body)
            return 200, self.service.release_segment(pid, seq, actor, role, zone, body)
        raise ApiError(404, "not_found", "接口不存在")
    def handle_request(self, method: str) -> None:
        parsed = urlparse(self.path)
        try:
            if method == "GET" and parsed.path == "/":
                raw = (self.web_root / "index.html").read_bytes(); self.send_response(200); self.send_header("Content-Type", "text/html; charset=utf-8"); self.send_header("Content-Length", str(len(raw))); self.end_headers(); self.wfile.write(raw); return
            status, payload = self.get_api(parsed.path) if method == "GET" else self.post_api(parsed.path); send_json(self, status, payload)
        except ApiError as exc:
            payload = {"error": exc.code, "message": exc.message}
            if exc.details is not None: payload["details"] = exc.details
            send_json(self, exc.status, payload)
        except Exception as exc: print(f"unhandled error: {exc!r}"); send_json(self, 500, {"error": "internal_error", "message": str(exc)})
    def do_GET(self) -> None: self.handle_request("GET")
    def do_POST(self) -> None: self.handle_request("POST")


def create_server(db_path: str | Path, host: str = "127.0.0.1", port: int = PORT) -> ThreadingHTTPServer:
    service = DroneAirspaceService(db_path); handler = type("DroneHandler", (Handler,), {"service": service, "web_root": Path(__file__).resolve().parent / "static"}); return ThreadingHTTPServer((host, port), handler)


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--host", default="127.0.0.1"); parser.add_argument("--port", type=int, default=PORT); parser.add_argument("--db", default=os.environ.get("DRONE_DB", "drone_airspace.db")); args = parser.parse_args()
    server = create_server(args.db, args.host, args.port); print(f"drone-airspace listening on http://{args.host}:{args.port}")
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()

if __name__ == "__main__": main()
