# 无人机飞行计划审批与空域协调系统

标准库独立项目。系统记录运营方计划、航线、载荷、高度、人口风险和应急方案，检查临时禁飞区、高度范围、人口风险以及相邻有效计划冲突。审核结果支持离线编号幂等回传，计划变更会使原批准失效并生成通知。

跨区低空走廊按协调区逐段流转：计划进入某区前，由该区值班员按当时限制和容量确认接收；接收凭证绑定计划版本（`V-P{计划}-R{版本}-S{序号}-{区}`）。只有上一区放行后下一区才能确认；两区同时确认同一槽位时先写入者生效，后到者收到 `handoff_conflict` 并看到当前所在区。任一区限制变化，后续未确认凭证作废（计划进入 `rescheduling`），已确认凭证保留快照、放行时若依据已变则被拦截。

## 运行

```bash
python3 app.py --db drone_airspace.db
```

默认监听 `127.0.0.1:8205`，首页 `/`，健康检查 `/health`。

身份头为 `X-User-Id`、`X-Role`；运营方还需 `X-Operator`，区段接收/放行值班员需带 `X-Zone-Code`（指挥官可跨区，不带）。角色：`viewer`、`operator`、`airspace_reviewer`、`commander`、`auditor`。

## 主要接口

- `POST /api/zones`、`GET /api/zones`：维护协调区（代码、顺序序号、矩形范围、容量 `capacity`）。
- `POST /api/restrictions`：新增临时限制或禁飞区；同一事务内作废命中区及下游的未确认凭证，返回 `voided_segments`。
- `POST /api/plans`：创建飞行计划。
- `POST /api/plans/{id}/submit`：提交时按航线与协调区自动切区（矩形裁剪 + 参数化时间窗）；无法唯一判定（覆盖不全/重叠/重回）则进入人工队列。
- `POST /api/plans/{id}/backfill`：旧计划按航线与限制范围回填区段；判不了的入人工队列，人工可用 `{"zones":["A","B"]}` 指定途经区序列强制落地。
- `POST /api/plans/{id}/segments/{seq}/accept|release`：逐区接收/放行，体中带按区编号的 `offline_id`（只在本区内唯一），接收支持 `expected_revision`。
- `POST /api/plans/{id}/sync`：断线回传批量合并，逐条幂等、独立成败（`finished` 给出已完成区段），只续办未完成区段。
- `GET /api/plans/{id}/check`：检查硬约束和相邻交通冲突。
- `POST /api/plans/{id}/submit`、`approve`、`reject`：提交和整体审核（旧流程保留）；审核使用 `offline_id` 保证断网重连幂等。
- `POST /api/plans/{id}/change`、`cancel`：版本化变更与取消。变更使旧版本 pending 凭证作废、accepted 凭证 superseded 并释放容量，需重走走廊；并生成通知。
- `GET /api/notifications`、`POST /api/expire`：通知与到期处理。通知携带唯一 `dedupe_key`，服务重启不重复产生。
- `GET /api/state`：按角色返回计划、区段、限制、协调区和人工队列。

## 区段状态

`pending`（待确认）→ `accepted`（已接收，凭证生效并计入容量）→ `released`（已放行，释放容量）；`voided`（限制变化/拒绝/取消/换版作废）、`superseded`（新版本取代）。计划状态新增 `rescheduling`；最后一区放行后计划整体 `approved`。

## 测试

```bash
python3 -m unittest discover -s tests -v
```

`tests/test_flow.py` 覆盖原有整体审批；`tests/test_zones.py` 覆盖逐区顺序闸门、版本凭证、并发先写者赢、限制变化作废、容量跨区隔离、按区幂等回传、重启不重复通知/占位、旧计划回填与人工判定。

## 主要局限

空域几何使用经纬度矩形和航线包围盒近似（切区使用线段-矩形裁剪与参数化时间窗），不包含多边形、椭球距离、地形、实时遥测和完整间隔标准。紧急授权只能覆盖空域及交通冲突，不能绕过载荷与高度硬限制。容量按区段时间窗重叠的 `accepted` 占位计数，不包含真实流量与间隔算法。身份头、无签名离线审核以及单连接 + 事务全局锁的单机 SQLite 适合原型，生产环境需要 PKI、真实 GIS 引擎、多连接/多实例并发控制和跨机构事件总线。
