#!/usr/bin/env python3
"""Drone flight-plan approval and airspace coordination service (standard library only)."""
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


class ApiError(Exception):
    def __init__(self, status: int, code: str, message: str, details: Any = None):
        super().__init__(message); self.status, self.code, self.message, self.details = status, code, message, details


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


class Repository:
    def __init__(self, path: str | Path):
        self.path = str(path)
        self._local = threading.local()
        self._initialize(self.conn)

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.path, check_same_thread=False, isolation_level=None, timeout=10.0)
        conn.row_factory = sqlite3.Row
        conn.execute("PRAGMA foreign_keys=ON")
        conn.execute("PRAGMA journal_mode=WAL")
        return conn

    @property
    def conn(self) -> sqlite3.Connection:
        c = getattr(self._local, "conn", None)
        if c is None:
            c = self._connect(); self._local.conn = c
        return c

    def _initialize(self, conn: sqlite3.Connection) -> None:
        conn.executescript("""
        CREATE TABLE IF NOT EXISTS coordination_zones(
            id INTEGER PRIMARY KEY AUTOINCREMENT, code TEXT NOT NULL UNIQUE, name TEXT NOT NULL, order_index INTEGER NOT NULL,
            min_lon REAL NOT NULL, min_lat REAL NOT NULL, max_lon REAL NOT NULL, max_lat REAL NOT NULL,
            capacity INTEGER NOT NULL DEFAULT 1, active INTEGER NOT NULL DEFAULT 1, created_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS restrictions(
            id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL, kind TEXT NOT NULL, zone_id INTEGER REFERENCES coordination_zones(id),
            min_lon REAL NOT NULL, min_lat REAL NOT NULL, max_lon REAL NOT NULL, max_lat REAL NOT NULL, min_altitude REAL NOT NULL DEFAULT 0, max_altitude REAL NOT NULL,
            starts_at TEXT NOT NULL, ends_at TEXT NOT NULL, reason TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'active', created_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS flight_plans(
            id INTEGER PRIMARY KEY AUTOINCREMENT, operator_id TEXT NOT NULL, callsign TEXT NOT NULL, drone_model TEXT NOT NULL,
            payload_kg REAL NOT NULL, route_json TEXT NOT NULL, starts_at TEXT NOT NULL, ends_at TEXT NOT NULL, max_altitude REAL NOT NULL,
            population_risk INTEGER NOT NULL, emergency_plan TEXT NOT NULL, region TEXT NOT NULL, status TEXT NOT NULL DEFAULT 'draft',
            revision INTEGER NOT NULL DEFAULT 1, corridor_status TEXT NOT NULL DEFAULT 'none', corridor_note TEXT,
            created_by TEXT NOT NULL, created_at TEXT NOT NULL, updated_at TEXT NOT NULL,
            UNIQUE(operator_id,callsign,starts_at)
        );
        CREATE TABLE IF NOT EXISTS approvals(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), plan_revision INTEGER NOT NULL,
            reviewer TEXT NOT NULL, decision TEXT NOT NULL, reason TEXT NOT NULL, offline_id TEXT UNIQUE,
            override_kind TEXT, created_at TEXT NOT NULL
        );
        CREATE TABLE IF NOT EXISTS plan_segments(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), zone_id INTEGER NOT NULL REFERENCES coordination_zones(id),
            sequence INTEGER NOT NULL, status TEXT NOT NULL DEFAULT 'pending', plan_revision INTEGER NOT NULL,
            restriction_snapshot TEXT NOT NULL DEFAULT '[]', accepted_by TEXT, accepted_at TEXT, accept_offline_id TEXT,
            released_by TEXT, released_at TEXT, release_offline_id TEXT, invalid_reason TEXT,
            created_at TEXT NOT NULL, updated_at TEXT NOT NULL,
            UNIQUE(plan_id, sequence)
        );
        CREATE TABLE IF NOT EXISTS notifications(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER NOT NULL REFERENCES flight_plans(id), kind TEXT NOT NULL,
            message TEXT NOT NULL, created_at TEXT NOT NULL, delivered INTEGER NOT NULL DEFAULT 0, dedup_key TEXT
        );
        CREATE TABLE IF NOT EXISTS audit_log(
            id INTEGER PRIMARY KEY AUTOINCREMENT, plan_id INTEGER, actor TEXT NOT NULL, role TEXT NOT NULL, action TEXT NOT NULL,
            detail_json TEXT NOT NULL, created_at TEXT NOT NULL
        );
        CREATE UNIQUE INDEX IF NOT EXISTS idx_seg_one_accepted ON plan_segments(plan_id) WHERE status='accepted';
        CREATE UNIQUE INDEX IF NOT EXISTS idx_seg_accept_offline ON plan_segments(accept_offline_id) WHERE accept_offline_id IS NOT NULL;
        CREATE UNIQUE INDEX IF NOT EXISTS idx_seg_release_offline ON plan_segments(release_offline_id) WHERE release_offline_id IS NOT NULL;
        CREATE UNIQUE INDEX IF NOT EXISTS idx_notifications_dedup ON notifications(plan_id, kind, dedup_key) WHERE dedup_key IS NOT NULL;
        """)
        for table, column, ddl in (
            ("flight_plans", "corridor_status", "TEXT NOT NULL DEFAULT 'none'"),
            ("flight_plans", "corridor_note", "TEXT"),
            ("restrictions", "zone_id", "INTEGER REFERENCES coordination_zones(id)"),
            ("notifications", "dedup_key", "TEXT"),
        ):
            cols = [r[1] for r in conn.execute(f"PRAGMA table_info({table})")]
            if column not in cols:
                conn.execute(f"ALTER TABLE {table} ADD COLUMN {column} {ddl}")

    @contextmanager
    def tx(self):
        self.conn.execute("BEGIN IMMEDIATE")
        try: yield self.conn; self.conn.execute("COMMIT")
        except Exception: self.conn.execute("ROLLBACK"); raise

    @staticmethod
    def audit(conn: sqlite3.Connection, plan_id: int | None, actor: str, role: str, action: str, detail: dict[str, Any]) -> None:
        conn.execute("INSERT INTO audit_log(plan_id,actor,role,action,detail_json,created_at) VALUES(?,?,?,?,?,?)",
                     (plan_id, actor, role, action, json.dumps(detail, ensure_ascii=False, sort_keys=True), iso()))

    @staticmethod
    def notify(conn: sqlite3.Connection, plan_id: int, kind: str, message: str) -> None:
        conn.execute("INSERT INTO notifications(plan_id,kind,message,created_at) VALUES(?,?,?,?)", (plan_id, kind, message, iso()))

    @staticmethod
    def notify_once(conn: sqlite3.Connection, plan_id: int, kind: str, message: str, dedup_key: str) -> None:
        """Idempotent notify: a (plan, kind, dedup_key) is inserted at most once, so a service restart never re-notifies."""
        conn.execute("INSERT OR IGNORE INTO notifications(plan_id,kind,message,created_at,dedup_key) VALUES(?,?,?,?,?)",
                     (plan_id, kind, message, iso(), dedup_key))


class DroneAirspaceService:
    def __init__(self, path: str | Path): self.repo = Repository(path)

    @staticmethod
    def identity(headers: Any) -> tuple[str, str, str]:
        actor, role, operator = headers.get("X-User-Id", "").strip(), headers.get("X-Role", "").strip(), headers.get("X-Operator", "").strip()
        if not actor or role not in ROLES: raise ApiError(401, "unauthorized", "需要 X-User-Id 和有效 X-Role")
        if role == "operator" and not operator: raise ApiError(401, "operator_required", "运营方角色必须提供 X-Operator")
        return actor, role, operator

    @staticmethod
    def _dict(row: sqlite3.Row | None) -> dict[str, Any] | None: return dict(row) if row else None

    def create_restriction(self, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "restriction_forbidden", "只有空域审核员或指挥官可以维护限制")
        name, kind, reason = str(body.get("name", "")).strip(), str(body.get("kind", "")).strip(), str(body.get("reason", "")).strip()
        if kind not in {"no_fly", "temporary_limit"} or not name or not reason: raise ApiError(400, "invalid_restriction", "名称、类型和原因必填")
        zone_id = body.get("zone_id")
        if zone_id is not None and (not isinstance(zone_id, int) or zone_id <= 0): raise ApiError(400, "invalid_restriction", "zone_id 必须为正整数")
        try:
            min_lon, min_lat, max_lon, max_lat = map(float, (body.get("min_lon"), body.get("min_lat"), body.get("max_lon"), body.get("max_lat")))
            min_alt, max_alt = float(body.get("min_altitude", 0)), float(body.get("max_altitude"))
        except (TypeError, ValueError): raise ApiError(400, "invalid_restriction", "空域范围和高度必须为数字")
        start, end = parse_time(body.get("starts_at")), parse_time(body.get("ends_at"))
        if min_lon >= max_lon or min_lat >= max_lat or min_alt < 0 or max_alt <= min_alt or end <= start:
            raise ApiError(400, "invalid_restriction", "空域范围、高度或时间无效")
        with self.repo.tx() as conn:
            if zone_id is not None and not conn.execute("SELECT id FROM coordination_zones WHERE id=?", (zone_id,)).fetchone():
                raise ApiError(400, "zone_not_found", f"协调区 {zone_id} 不存在")
            cur = conn.execute("""INSERT INTO restrictions(name,kind,zone_id,min_lon,min_lat,max_lon,max_lat,min_altitude,max_altitude,starts_at,ends_at,reason,created_at)
                                  VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                               (name, kind, zone_id, min_lon, min_lat, max_lon, max_lat, min_alt, max_alt, iso(start), iso(end), reason, iso()))
            row = conn.execute("SELECT * FROM restrictions WHERE id=?", (cur.lastrowid,)).fetchone()
            self._invalidate_corridor_for_restriction(conn, row)
            return dict(row)

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

    def get_plan(self, plan_id: int, role: str, operator: str = "") -> dict[str, Any]:
        conn = self.repo.conn; row = self._plan_row(conn, plan_id)
        if role == "operator" and row["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能查看其他运营方计划")
        result = dict(row); result["route"] = json.loads(result.pop("route_json")); result["route_bbox"] = route_bbox(result["route"])
        if role == "viewer":
            result = {key: result[key] for key in ("id", "callsign", "starts_at", "ends_at", "max_altitude", "region", "status", "corridor_status", "valid_until" if "valid_until" in result else "updated_at")}
        if role in {"airspace_reviewer", "commander", "auditor"}: result["approvals"] = [dict(r) for r in conn.execute("SELECT * FROM approvals WHERE plan_id=? ORDER BY id", (plan_id,))]
        return result

    def submit(self, plan_id: int, actor: str, role: str, operator: str, body: dict[str, Any]) -> dict[str, Any]:
        if role != "operator": raise ApiError(403, "submit_forbidden", "只有运营方可以提交计划")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if plan["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能提交其他运营方计划")
            if plan["status"] == "submitted": return {"plan": self.get_plan(plan_id, role, operator), "idempotent": True}
            if plan["status"] not in {"draft", "rejected"}: raise ApiError(409, "invalid_transition", "当前状态不能提交")
            if parse_time(plan["starts_at"]) <= utcnow(): raise ApiError(409, "plan_expired", "计划起飞时间已过")
            conn.execute("UPDATE flight_plans SET status='submitted',updated_at=? WHERE id=?", (iso(), plan_id))
            Repository.audit(conn, plan_id, actor, role, "plan_submitted", {"revision": plan["revision"]})
            return {"plan": self.get_plan(plan_id, role, operator), "idempotent": False}

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
            Repository.notify(conn, plan_id, "approved", f"飞行计划 {plan['callsign']} 已批准")
            self.ensure_segments(conn, plan)
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
            conn.execute("UPDATE flight_plans SET status='rejected',updated_at=? WHERE id=?", (iso(), plan_id))
            Repository.audit(conn, plan_id, actor, role, "plan_rejected", {"reason": reason, "offline_id": offline_id})
            Repository.notify(conn, plan_id, "rejected", f"飞行计划 {plan['callsign']} 被拒绝：{reason}")
            return {"plan": self.get_plan(plan_id, role, ""), "idempotent": False}

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
            conn.execute("UPDATE plan_segments SET status='invalidated', invalid_reason='plan_revision_changed', updated_at=? WHERE plan_id=? AND status!='released'", (iso(), plan_id))
            conn.execute("UPDATE flight_plans SET corridor_status='rearranging', updated_at=? WHERE id=?", (iso(), plan_id))
            Repository.audit(conn, plan_id, actor, role, "plan_changed", {"from_revision": expected, "to_revision": revision, "previous_status": plan["status"]})
            if plan["status"] == "approved": Repository.notify(conn, plan_id, "approval_invalidated", f"飞行计划 {plan['callsign']} 已修改，原批准自动失效")
            else: Repository.notify(conn, plan_id, "changed", f"飞行计划 {plan['callsign']} 已更新，需重新提交审核")
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
            conn.execute("UPDATE flight_plans SET status='canceled',updated_at=? WHERE id=?", (iso(), plan_id))
            Repository.audit(conn, plan_id, actor, role, "plan_canceled", {"reason": reason})
            Repository.notify(conn, plan_id, "canceled", f"飞行计划 {plan['callsign']} 已取消：{reason}")
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
                Repository.notify(conn, row["id"], "expired", f"飞行计划 {row['callsign']} 已过期")
        return {"expired": len(rows)}

    def state(self, role: str, operator: str) -> dict[str, Any]:
        conn = self.repo.conn
        if role == "operator": rows = conn.execute("SELECT * FROM flight_plans WHERE operator_id=? ORDER BY id DESC", (operator,))
        elif role in {"airspace_reviewer", "commander", "auditor"}: rows = conn.execute("SELECT * FROM flight_plans ORDER BY id DESC")
        else: rows = conn.execute("SELECT * FROM flight_plans WHERE status='approved' ORDER BY id DESC")
        plans = []
        for row in rows:
            item = self.get_plan(row["id"], role, operator); plans.append(item)
        restrictions = [dict(r) for r in conn.execute("SELECT * FROM restrictions WHERE status='active' ORDER BY id DESC")] if role in {"airspace_reviewer", "commander", "auditor"} else []
        return {"plans": plans, "restrictions": restrictions, "server_time": iso()}

    # ---- 协调区与走廊交接 ----
    def create_zone(self, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "zone_forbidden", "只有空域审核员或指挥官可以维护协调区")
        code, name = str(body.get("code", "")).strip(), str(body.get("name", "")).strip()
        if not code or not name: raise ApiError(400, "invalid_zone", "区编号和名称必填")
        try:
            min_lon, min_lat, max_lon, max_lat = map(float, (body.get("min_lon"), body.get("min_lat"), body.get("max_lon"), body.get("max_lat")))
            capacity = int(body.get("capacity", 1)); order_index = int(body.get("order_index", 0))
        except (TypeError, ValueError): raise ApiError(400, "invalid_zone", "范围、容量和顺序必须为数字")
        if min_lon >= max_lon or min_lat >= max_lat or capacity < 1: raise ApiError(400, "invalid_zone", "协调区范围或容量无效")
        with self.repo.tx() as conn:
            if not order_index:
                order_index = conn.execute("SELECT COALESCE(MAX(order_index),0)+1 AS n FROM coordination_zones").fetchone()["n"]
            try:
                cur = conn.execute("""INSERT INTO coordination_zones(code,name,order_index,min_lon,min_lat,max_lon,max_lat,capacity,created_at)
                                      VALUES(?,?,?,?,?,?,?,?,?)""", (code, name, order_index, min_lon, min_lat, max_lon, max_lat, capacity, iso()))
            except sqlite3.IntegrityError as exc: raise ApiError(409, "zone_duplicate", "区编号已存在") from exc
            Repository.audit(conn, None, actor, role, "zone_created", {"zone_id": cur.lastrowid, "code": code, "order_index": order_index})
            return dict(conn.execute("SELECT * FROM coordination_zones WHERE id=?", (cur.lastrowid,)).fetchone())

    def list_zones(self, role: str) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander", "auditor", "operator", "viewer"}: raise ApiError(403, "zone_forbidden", "无权查看协调区")
        conn = self.repo.conn
        zones = [dict(r) for r in conn.execute("SELECT * FROM coordination_zones WHERE active=1 ORDER BY order_index,id")]
        for z in zones:
            z["occupied"] = conn.execute("SELECT COUNT(*) AS n FROM plan_segments WHERE zone_id=? AND status='accepted'", (z["id"],)).fetchone()["n"]
        return {"zones": zones}

    def set_restriction_status(self, restriction_id: int, actor: str, role: str, status: str) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "restriction_forbidden", "只有空域审核员或指挥官可以维护限制")
        if status not in {"active", "inactive"}: raise ApiError(400, "invalid_status", "状态必须为 active 或 inactive")
        with self.repo.tx() as conn:
            row = conn.execute("SELECT * FROM restrictions WHERE id=?", (restriction_id,)).fetchone()
            if not row: raise ApiError(404, "restriction_not_found", "限制不存在")
            if row["status"] == status: return {"restriction": dict(row), "idempotent": True}
            conn.execute("UPDATE restrictions SET status=? WHERE id=?", (status, restriction_id))
            Repository.audit(conn, restriction_id, actor, role, "restriction_status_changed", {"status": status})
            if status == "active": self._invalidate_corridor_for_restriction(conn, row)
            return {"restriction": dict(conn.execute("SELECT * FROM restrictions WHERE id=?", (restriction_id,)).fetchone()), "idempotent": False}

    @staticmethod
    def _point_zones(lon: float, lat: float, zones: list[sqlite3.Row]) -> list[sqlite3.Row]:
        # 半开区间避免相邻协调区在边界上重复计入
        return [z for z in zones if z["min_lon"] <= lon < z["max_lon"] and z["min_lat"] <= lat < z["max_lat"]]

    def _route_traversal(self, conn: sqlite3.Connection, route: list[list[float]]) -> tuple[list[dict[str, Any]] | None, str | None]:
        zones = list(conn.execute("SELECT * FROM coordination_zones WHERE active=1 ORDER BY order_index,id"))
        if not zones: return None, "no_zones"
        first_index: dict[int, float] = {}
        samples = 12  # 沿相邻航线点插值采样，确保跨越的协调区都被计入
        for i in range(len(route) - 1):
            lon0, lat0, lon1, lat1 = route[i][0], route[i][1], route[i + 1][0], route[i + 1][1]
            for k in range(samples + 1):
                t = k / samples
                lon, lat = lon0 + (lon1 - lon0) * t, lat0 + (lat1 - lat0) * t
                containing = self._point_zones(lon, lat, zones)
                if len(containing) > 1: return None, "ambiguous_zones"
                if containing:
                    zid = containing[0]["id"]; frac = i + t
                    if zid not in first_index or frac < first_index[zid]: first_index[zid] = frac
        if not first_index: return None, "route_outside_corridor"
        ordered = sorted(first_index.items(), key=lambda kv: kv[1])
        return [{"zone_id": zid, "sequence": idx + 1} for idx, (zid, _) in enumerate(ordered)], None

    def _rebuild_segments(self, conn: sqlite3.Connection, plan: sqlite3.Row) -> list[dict[str, Any]] | None:
        traversal, reason = self._route_traversal(conn, self._route(plan))
        conn.execute("DELETE FROM plan_segments WHERE plan_id=?", (plan["id"],))
        if traversal is None:
            conn.execute("UPDATE flight_plans SET corridor_status='manual', corridor_note=?, updated_at=? WHERE id=?", (reason, iso(), plan["id"]))
            return None
        for t in traversal:
            conn.execute("""INSERT INTO plan_segments(plan_id,zone_id,sequence,status,plan_revision,restriction_snapshot,created_at,updated_at)
                            VALUES(?,?,?,'pending',?,'[]',?,?)""", (plan["id"], t["zone_id"], t["sequence"], plan["revision"], iso(), iso()))
        conn.execute("UPDATE flight_plans SET corridor_status='active', corridor_note=NULL, updated_at=? WHERE id=?", (iso(), plan["id"]))
        return traversal

    def ensure_segments(self, conn: sqlite3.Connection, plan: sqlite3.Row) -> list[sqlite3.Row] | None:
        existing = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))
        if existing and plan["corridor_status"] in {"active", "completed"}: return existing
        if plan["corridor_status"] == "manual" and not existing: return None
        rebuilt = self._rebuild_segments(conn, plan)
        return None if rebuilt is None else list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))

    def backfill_all(self, actor: str, role: str) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "backfill_forbidden", "只有空域审核员或指挥官可以回填区段")
        with self.repo.tx() as conn:
            plans = list(conn.execute("SELECT * FROM flight_plans WHERE corridor_status IN ('none','manual') ORDER BY id"))
            backfilled: list[dict[str, Any]] = []; manual: list[dict[str, Any]] = []
            for plan in plans:
                result = self._rebuild_segments(conn, plan)
                if result is None:
                    note = conn.execute("SELECT corridor_note FROM flight_plans WHERE id=?", (plan["id"],)).fetchone()["corridor_note"]
                    manual.append({"plan_id": plan["id"], "callsign": plan["callsign"], "reason": note})
                else:
                    backfilled.append({"plan_id": plan["id"], "callsign": plan["callsign"], "segments": len(result)})
            Repository.audit(conn, None, actor, role, "corridor_backfill", {"backfilled": len(backfilled), "manual": len(manual)})
            return {"backfilled": backfilled, "manual": manual}

    def manual_assign(self, plan_id: int, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "manual_forbidden", "只有空域审核员或指挥官可以人工指定区段")
        zone_ids = body.get("zone_ids")
        if not isinstance(zone_ids, list) or not zone_ids or not all(isinstance(v, int) for v in zone_ids):
            raise ApiError(400, "invalid_segments", "zone_ids 必须为非空整数列表")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if plan["status"] != "approved": raise ApiError(409, "plan_not_approved", "计划必须先批准才能进入走廊")
            for zid in zone_ids:
                if not conn.execute("SELECT id FROM coordination_zones WHERE id=? AND active=1", (zid,)).fetchone():
                    raise ApiError(400, "zone_not_found", f"协调区 {zid} 不存在或未启用")
            conn.execute("DELETE FROM plan_segments WHERE plan_id=?", (plan["id"],))
            for idx, zid in enumerate(zone_ids, start=1):
                conn.execute("""INSERT INTO plan_segments(plan_id,zone_id,sequence,status,plan_revision,restriction_snapshot,created_at,updated_at)
                                VALUES(?,?,?,'pending',?,'[]',?,?)""", (plan["id"], zid, idx, plan["revision"], iso(), iso()))
            conn.execute("UPDATE flight_plans SET corridor_status='active', corridor_note='manual_assignment', updated_at=? WHERE id=?", (iso(), plan["id"]))
            Repository.audit(conn, plan_id, actor, role, "corridor_manual_assign", {"zone_ids": zone_ids})
            return self.corridor_state(plan_id, role, "")

    def _zone_restrictions(self, conn: sqlite3.Connection, zone: sqlite3.Row, plan: sqlite3.Row) -> list[dict[str, Any]]:
        zbox = (zone["min_lon"], zone["min_lat"], zone["max_lon"], zone["max_lat"])
        start, end = parse_time(plan["starts_at"]), parse_time(plan["ends_at"])
        out: list[dict[str, Any]] = []
        for r in conn.execute("SELECT * FROM restrictions WHERE status='active'"):
            rbox = (r["min_lon"], r["min_lat"], r["max_lon"], r["max_lat"])
            if not boxes_overlap(zbox, rbox): continue
            if not times_overlap(start, end, parse_time(r["starts_at"]), parse_time(r["ends_at"])): continue
            out.append({"restriction_id": r["id"], "name": r["name"], "kind": r["kind"], "reason": r["reason"]})
        return out

    def _segment_view(self, conn: sqlite3.Connection, seg: sqlite3.Row) -> dict[str, Any]:
        d = dict(seg)
        zone = conn.execute("SELECT id,code,name,order_index,capacity FROM coordination_zones WHERE id=?", (seg["zone_id"],)).fetchone()
        d["zone"] = dict(zone) if zone else None
        return d

    @staticmethod
    def _current_position(conn: sqlite3.Connection, plan: sqlite3.Row, segments: list[sqlite3.Row]) -> dict[str, Any] | None:
        for s in segments:
            if s["status"] in {"pending", "accepted", "invalidated"}:
                zone = conn.execute("SELECT id,code,name FROM coordination_zones WHERE id=?", (s["zone_id"],)).fetchone()
                return {"segment_id": s["id"], "sequence": s["sequence"], "zone": dict(zone) if zone else None, "status": s["status"]}
        return None

    def _position_details(self, conn: sqlite3.Connection, plan: sqlite3.Row, segments: list[sqlite3.Row], target: sqlite3.Row | None = None) -> dict[str, Any]:
        details: dict[str, Any] = {"plan_id": plan["id"], "revision": plan["revision"], "corridor_status": plan["corridor_status"]}
        current = self._current_position(conn, plan, segments)
        if current:
            details["current_position"] = current
        if target:
            zone = conn.execute("SELECT id,code,name FROM coordination_zones WHERE id=?", (target["zone_id"],)).fetchone()
            details["target_zone"] = dict(zone) if zone else None
            details["target_sequence"] = target["sequence"]
            details["target_status"] = target["status"]
        return details

    def _position_message(self, conn: sqlite3.Connection, plan: sqlite3.Row, segments: list[sqlite3.Row], target: sqlite3.Row) -> str:
        current = self._current_position(conn, plan, segments)
        tzone = conn.execute("SELECT code FROM coordination_zones WHERE id=?", (target["zone_id"],)).fetchone()
        tcode = tzone["code"] if tzone else "?"
        if current and current["zone"]:
            return f"计划当前在 {current['zone']['code']} 区（状态 {current['status']}），不能对 {tcode} 区执行该操作"
        return f"计划已完成走廊交接，不能对 {tcode} 区执行该操作"

    def corridor_state(self, plan_id: int, role: str, operator: str) -> dict[str, Any]:
        conn = self.repo.conn
        plan = self._plan_row(conn, plan_id)
        if role == "operator" and plan["operator_id"] != operator: raise ApiError(403, "plan_forbidden", "不能查看其他运营方计划")
        segments = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))
        views = [self._segment_view(conn, s) for s in segments]
        return {"plan_id": plan["id"], "callsign": plan["callsign"], "revision": plan["revision"], "corridor_status": plan["corridor_status"],
                "corridor_note": plan["corridor_note"], "current_position": self._current_position(conn, plan, segments),
                "segments": views, "server_time": iso()}

    def accept_segment(self, plan_id: int, seq: int, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "segment_forbidden", "只有空域审核员或指挥官可以确认接收")
        offline_id = str(body.get("offline_id", "")).strip()
        if not offline_id: raise ApiError(400, "offline_id_required", "offline_id 必填")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            if plan["status"] != "approved": raise ApiError(409, "plan_not_approved", "计划必须先批准才能进入走廊")
            segments = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))
            if not segments:
                self.ensure_segments(conn, plan)
                segments = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))
            target = next((s for s in segments if s["sequence"] == seq), None)
            if not target: raise ApiError(404, "segment_not_found", "区段不存在")
            zone = conn.execute("SELECT * FROM coordination_zones WHERE id=?", (target["zone_id"],)).fetchone()
            prior = conn.execute("SELECT * FROM plan_segments WHERE accept_offline_id=?", (offline_id,)).fetchone()
            if prior:
                if prior["plan_id"] == plan_id and prior["sequence"] == seq and prior["zone_id"] == target["zone_id"] and prior["status"] == "accepted":
                    return {"segment": self._segment_view(conn, target), "idempotent": True}
                raise ApiError(409, "offline_id_conflict", "该离线接收编号已用于其他交接")
            if target["status"] != "pending":
                raise ApiError(409, "segment_not_pending", self._position_message(conn, plan, segments, target), self._position_details(conn, plan, segments, target))
            if seq > 1:
                prev = next((s for s in segments if s["sequence"] == seq - 1), None)
                if not prev or prev["status"] != "released":
                    raise ApiError(409, "previous_not_released", self._position_message(conn, plan, segments, target), self._position_details(conn, plan, segments, target))
            other = next((s for s in segments if s["status"] == "accepted" and s["id"] != target["id"]), None)
            if other: raise ApiError(409, "plan_in_zone", self._position_message(conn, plan, segments, target), self._position_details(conn, plan, segments, target))
            expected = body.get("expected_revision")
            if expected is not None and expected != plan["revision"]: raise ApiError(409, "revision_conflict", "计划版本已变化，接收凭证不能套用")
            occupied = conn.execute("SELECT COUNT(*) AS n FROM plan_segments WHERE zone_id=? AND status='accepted'", (zone["id"],)).fetchone()["n"]
            if occupied >= zone["capacity"]: raise ApiError(409, "capacity_full", f"协调区 {zone['code']} 容量已满", {"zone": zone["code"], "capacity": zone["capacity"], "occupied": occupied})
            snapshot = self._zone_restrictions(conn, zone, plan)
            if snapshot: raise ApiError(409, "restriction_blocking", f"协调区 {zone['code']} 当前有限制，不能接收", snapshot)
            conn.execute("""UPDATE plan_segments SET status='accepted', plan_revision=?, restriction_snapshot=?, accepted_by=?, accepted_at=?, accept_offline_id=?, invalid_reason=NULL, updated_at=? WHERE id=?""",
                         (plan["revision"], json.dumps(snapshot, ensure_ascii=False), actor, iso(), offline_id, iso(), target["id"]))
            if plan["corridor_status"] != "active":
                conn.execute("UPDATE flight_plans SET corridor_status='active', updated_at=? WHERE id=?", (iso(), plan["id"]))
            Repository.audit(conn, plan_id, actor, role, "segment_accepted", {"sequence": seq, "zone": zone["code"], "offline_id": offline_id, "revision": plan["revision"]})
            Repository.notify_once(conn, plan_id, "segment_accepted", f"飞行计划 {plan['callsign']} 已由 {zone['code']} 区接收", f"accept:{plan_id}:{zone['id']}:{seq}")
            target = conn.execute("SELECT * FROM plan_segments WHERE id=?", (target["id"],)).fetchone()
            return {"segment": self._segment_view(conn, target), "idempotent": False}

    def release_segment(self, plan_id: int, seq: int, actor: str, role: str, body: dict[str, Any]) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "segment_forbidden", "只有空域审核员或指挥官可以放行")
        offline_id = str(body.get("offline_id", "")).strip()
        if not offline_id: raise ApiError(400, "offline_id_required", "offline_id 必填")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            segments = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))
            target = next((s for s in segments if s["sequence"] == seq), None)
            if not target: raise ApiError(404, "segment_not_found", "区段不存在")
            zone = conn.execute("SELECT * FROM coordination_zones WHERE id=?", (target["zone_id"],)).fetchone()
            prior = conn.execute("SELECT * FROM plan_segments WHERE release_offline_id=?", (offline_id,)).fetchone()
            if prior:
                if prior["plan_id"] == plan_id and prior["sequence"] == seq and prior["zone_id"] == target["zone_id"] and prior["status"] == "released":
                    return {"segment": self._segment_view(conn, target), "idempotent": True}
                raise ApiError(409, "offline_id_conflict", "该离线放行编号已用于其他交接")
            if target["status"] != "accepted":
                raise ApiError(409, "segment_not_accepted", self._position_message(conn, plan, segments, target), self._position_details(conn, plan, segments, target))
            conn.execute("UPDATE plan_segments SET status='released', released_by=?, released_at=?, release_offline_id=?, updated_at=? WHERE id=?",
                         (actor, iso(), offline_id, iso(), target["id"]))
            if all(s["status"] == "released" or s["id"] == target["id"] for s in segments):
                conn.execute("UPDATE flight_plans SET corridor_status='completed', updated_at=? WHERE id=?", (iso(), plan["id"]))
            Repository.audit(conn, plan_id, actor, role, "segment_released", {"sequence": seq, "zone": zone["code"], "offline_id": offline_id})
            Repository.notify_once(conn, plan_id, "segment_released", f"飞行计划 {plan['callsign']} 已由 {zone['code']} 区放行", f"release:{plan_id}:{zone['id']}:{seq}")
            target = conn.execute("SELECT * FROM plan_segments WHERE id=?", (target["id"],)).fetchone()
            return {"segment": self._segment_view(conn, target), "idempotent": False}

    def rearrange(self, plan_id: int, actor: str, role: str) -> dict[str, Any]:
        if role not in {"airspace_reviewer", "commander"}: raise ApiError(403, "rearrange_forbidden", "只有空域审核员或指挥官可以退回重排")
        with self.repo.tx() as conn:
            plan = self._plan_row(conn, plan_id)
            segments = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (plan["id"],)))
            invalidated = [s for s in segments if s["status"] == "invalidated"]
            if not invalidated and plan["corridor_status"] != "rearranging":
                return {"plan_id": plan_id, "idempotent": True, "rearranged": 0, "corridor_status": plan["corridor_status"]}
            for s in invalidated:
                conn.execute("""UPDATE plan_segments SET status='pending', plan_revision=?, restriction_snapshot='[]', accepted_by=NULL, accepted_at=NULL, accept_offline_id=NULL,
                                invalid_reason=NULL, updated_at=? WHERE id=?""", (plan["revision"], iso(), s["id"]))
            conn.execute("UPDATE flight_plans SET corridor_status='active', updated_at=? WHERE id=?", (iso(), plan["id"]))
            Repository.audit(conn, plan_id, actor, role, "corridor_rearranged", {"count": len(invalidated)})
            Repository.notify_once(conn, plan_id, "corridor_rearranged", f"飞行计划 {plan['callsign']} 已退回重排，后续区段重新确认", f"rearrange:{plan_id}")
            return {"plan_id": plan_id, "idempotent": False, "rearranged": len(invalidated), "corridor_status": "active"}

    def _invalidate_corridor_for_restriction(self, conn: sqlite3.Connection, restriction: sqlite3.Row) -> None:
        rbox = (restriction["min_lon"], restriction["min_lat"], restriction["max_lon"], restriction["max_lat"])
        zones = [z for z in conn.execute("SELECT * FROM coordination_zones WHERE active=1 ORDER BY order_index")
                 if boxes_overlap((z["min_lon"], z["min_lat"], z["max_lon"], z["max_lat"]), rbox)]
        for z in zones:
            plan_ids = [r["plan_id"] for r in conn.execute("SELECT DISTINCT plan_id FROM plan_segments WHERE zone_id=? AND status IN ('pending','accepted')", (z["id"],))]
            for pid in plan_ids:
                plan = self._plan_row(conn, pid)
                if plan["corridor_status"] not in {"active", "rearranging"}: continue
                zseg = conn.execute("SELECT * FROM plan_segments WHERE plan_id=? AND zone_id=?", (pid, z["id"])).fetchone()
                if not zseg: continue
                segs = list(conn.execute("SELECT * FROM plan_segments WHERE plan_id=? ORDER BY sequence", (pid,)))
                changed = False
                for s in segs:
                    if s["sequence"] >= zseg["sequence"] and s["status"] == "pending":
                        conn.execute("UPDATE plan_segments SET status='invalidated', invalid_reason='restriction_changed', updated_at=? WHERE id=?", (iso(), s["id"]))
                        changed = True
                if changed:
                    conn.execute("UPDATE flight_plans SET corridor_status='rearranging', updated_at=? WHERE id=?", (iso(), pid))
                    Repository.notify_once(conn, pid, "corridor_rearrange",
                                           f"飞行计划 {plan['callsign']} 因 {z['code']} 区限制变化，后续未确认凭证作废并退回重排",
                                           f"restriction:{restriction['id']}:zone:{z['id']}")


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
        if path == "/api/state": return 200, self.service.state(role, operator)
        if path == "/api/notifications": return 200, self.service.notifications(actor, role, operator)
        if path == "/api/zones": return 200, self.service.list_zones(role)
        parts = [p for p in path.split("/") if p]
        if len(parts) == 3 and parts[:2] == ["api", "plans"] and parts[2].isdigit(): return 200, self.service.get_plan(int(parts[2]), role, operator)
        if len(parts) == 4 and parts[:2] == ["api", "plans"] and parts[2].isdigit() and parts[3] == "check": return 200, self.service.check_conflicts(int(parts[2]), role, operator)
        if len(parts) == 4 and parts[:2] == ["api", "plans"] and parts[2].isdigit() and parts[3] == "corridor": return 200, self.service.corridor_state(int(parts[2]), role, operator)
        raise ApiError(404, "not_found", "接口不存在")
    def post_api(self, path: str) -> tuple[int, Any]:
        actor, role, operator = self.service.identity(self.headers); body = self.body(); parts = [p for p in path.split("/") if p]
        if path == "/api/restrictions": return 201, self.service.create_restriction(actor, role, body)
        if path == "/api/plans": return 201, self.service.create_plan(actor, role, operator, body)
        if path == "/api/zones": return 201, self.service.create_zone(actor, role, body)
        if path == "/api/admin/backfill": return 200, self.service.backfill_all(actor, role)
        if path == "/api/expire": return 200, self.service.expire_plans(actor, role)
        if len(parts) == 4 and parts[:2] == ["api", "restrictions"] and parts[2].isdigit() and parts[3] in {"deactivate", "activate"}:
            return 200, self.service.set_restriction_status(int(parts[2]), actor, role, "inactive" if parts[3] == "deactivate" else "active")
        if len(parts) == 4 and parts[:2] == ["api", "plans"] and parts[2].isdigit():
            pid, action = int(parts[2]), parts[3]
            routes = {
                "submit": lambda: self.service.submit(pid, actor, role, operator, body),
                "approve": lambda: self.service.approve(pid, actor, role, body),
                "reject": lambda: self.service.reject(pid, actor, role, body),
                "change": lambda: self.service.change(pid, actor, role, operator, body),
                "cancel": lambda: self.service.cancel(pid, actor, role, operator, body),
            }
            if action in routes: return 200, routes[action]()
        if len(parts) == 5 and parts[:2] == ["api", "plans"] and parts[2].isdigit() and parts[3] == "corridor":
            pid = int(parts[2])
            if parts[4] == "rearrange": return 200, self.service.rearrange(pid, actor, role)
            if parts[4] == "manual": return 200, self.service.manual_assign(pid, actor, role, body)
        if len(parts) == 7 and parts[:2] == ["api", "plans"] and parts[2].isdigit() and parts[3] == "corridor" and parts[4] == "segments" and parts[6] in {"accept", "release"}:
            pid, seq, action = int(parts[2]), int(parts[5]), parts[6]
            if action == "accept": return 200, self.service.accept_segment(pid, seq, actor, role, body)
            return 200, self.service.release_segment(pid, seq, actor, role, body)
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
