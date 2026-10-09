"""
Qt 라이브 플릿 뷰어 (표시 전용)
==============================

matplotlib 버전(`tools/map_viewer.py`)을 대체하는 실시간 뷰어. PySide6 +
QGraphicsView 기반 — GoCart Viz 같은 로봇 시각화 툴의 표준 구성이다.

**이 모듈도 로봇을 시뮬레이션하지 않는다.** 여전히 순수 표시 레이어다.
matplotlib 버전과 똑같이 로봇이 보고한 값만 그리고, 아무것도 발행하지 않는다.
MQTT 구독/파싱은 `tools/fleet_monitor.py` 의 `FleetMonitor` 를 그대로 재사용한다
(그쪽은 처음부터 렌더러에 의존하지 않게 짜여 있었다 — `update_robot()` /
`set_connection()` 두 메서드만 호출한다).

라이브로 보려면:
    python tools/fleet_monitor_qt.py maps/warehouse/warehouse.json

matplotlib 버전 대비 얻는 것
    - 확대/축소/이동이 Qt 기본 기능 (QGraphicsView) — 직접 짤 필요 없다
    - 창 크기 조절 시 그냥 더 넓은 캔버스가 보인다 (matplotlib 의
      aspect="datalim" 같은 우회가 필요 없다)
    - 로봇/경로가 진짜 객체(QGraphicsItem)라서 좌표만 갱신하면 된다
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QBrush, QColor, QFont, QIcon, QImage, QPainterPath, QPen, QPixmap, QPolygonF,
)
from PySide6.QtWidgets import (
    QGraphicsPathItem, QGraphicsPixmapItem, QGraphicsPolygonItem,
    QGraphicsRectItem, QGraphicsScene, QGraphicsView,
    QLabel, QMainWindow,
)

from common.map_model import MapModel

ICON_PATH = Path(__file__).resolve().parent.parent / "assets" / "icons" / "fleet_viewer_icon.png"

# matplotlib 버전과 같은 팔레트를 쓴다 (일관성)
CELL_COLORS = [QColor("#1c1c1e"), QColor("#9aa0a6"), QColor("#ffffff")]  # OCCUPIED/UNKNOWN/FREE
ROBOT_PALETTE = [
    "#e5484d", "#0090ff", "#30a46c", "#f5a623", "#8e4ec6",
    "#e93d82", "#12a594", "#d6409f", "#f76b15", "#3e63dd",
    "#46a758", "#ab4aba", "#e54666", "#0d9488", "#ca8a04",
    "#7c3aed", "#dc2626", "#0284c7", "#65a30d", "#c026d3",
]

STALE_AFTER = 2.0          # 이 시간 동안 상태 보고가 없으면 흐리게
EVENT_LOG_MAX_LINES = 14
PATH_TICK_SPACING_PX = 14  # 경로선 눈금 간격 (화면 픽셀 아니라 scene 단위 — 아래 참고)

# zone 타입별 색 — vendor zone_meta.json 을 그리는 용도 (표시 전용, 로봇 내비게이션엔
# 관여 안 한다 — 그쪽은 fms_server 가 grid 에 구워서 각 로봇이 직접 받는다)
ZONE_COLORS = {
    "prohibit": QColor(255, 59, 48),   # 진입 금지 — 빨강
    "slow": QColor(255, 159, 10),      # 감속 구역 — 주황
    "oneway": QColor(10, 132, 255),    # 일방통행 — 파랑
}
ZONE_DEFAULT_COLOR = QColor(154, 160, 166)  # 모르는 타입 — 회색


# -- 좌표 변환 ----------------------------------------------------------------
# scene 좌표계 = 이미지 픽셀 좌표계 그대로 쓴다 (1 scene unit = 1 px = resolution m).
# common/map_model.py 가 이제 world y 도 아래로 증가하는 규약(origin=좌상단, 뒤집기 없음)
# 을 쓰므로 QGraphicsView 의 기본 y-down 좌표계와 방향이 그대로 맞아 떨어진다 —
# 뒤집을 필요가 없다. world_to_pixel/pixel_to_world 와 동일한 식의 연속값 버전이다.

def world_to_scene(meta, x: float, y: float) -> QPointF:
    sx = (x - meta.origin_x) / meta.resolution
    sy = (y - meta.origin_y) / meta.resolution
    return QPointF(sx, sy)


def scene_to_world(meta, sx: float, sy: float) -> tuple[float, float]:
    x = meta.origin_x + sx * meta.resolution
    y = meta.origin_y + sy * meta.resolution
    return x, y


@dataclass
class RobotView:
    """뷰어가 아는 로봇 하나. 전부 로봇이 보고한 값이다 (matplotlib 버전과 동일 계약)."""

    robot_id: str
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    velocity: float = 0.0
    width: float = 0.5
    length: float = 0.7
    state: str = "?"
    battery: Optional[float] = None
    connection: str = "?"
    order_id: Optional[str] = None
    remaining_nodes: int = 0
    errors: list[str] = field(default_factory=list)
    last_seen: float = 0.0
    planned_path: list[tuple[float, float]] = field(default_factory=list)
    detected_obstacles: list[tuple[float, float, float, float]] = field(default_factory=list)
    has_pose: bool = False

    def is_stale(self, now: float) -> bool:
        return self.has_pose and (now - self.last_seen) > STALE_AFTER


class _OutlinedLabel(QGraphicsPathItem):
    """
    검은 테두리 + 흰색 채우기 텍스트. 흰 배경(FREE 셀) 위에서 흰 글자가 안 보이던
    문제 때문에 도입 — QGraphicsSimpleTextItem 은 외곽선을 못 그려서, 텍스트를
    QPainterPath 로 만들어 pen(외곽선)/brush(채우기)를 따로 준다. 배경이 검든
    희든 항상 보인다.
    """

    def __init__(self) -> None:
        super().__init__()
        self._font = QFont()
        outline = QPen(QColor("black"))
        outline.setWidthF(2.2)
        outline.setCosmetic(True)
        outline.setJoinStyle(Qt.RoundJoin)
        self.setPen(outline)
        self.setBrush(QBrush(QColor("white")))

    def font(self) -> QFont:
        return self._font

    def setFont(self, font: QFont) -> None:
        self._font = font

    def setText(self, text: str) -> None:
        path = QPainterPath()
        path.addText(0, 0, self._font, text)
        self.setPath(path)


class _RobotItem:
    """로봇 하나를 그리는 QGraphicsItem 묶음 (그룹 아님 — 라벨은 회전시키면 안 돼서 따로 둔다)."""

    def __init__(self, scene: QGraphicsScene, color: QColor):
        pen = QPen(QColor("white"))
        pen.setWidthF(1.4)
        pen.setCosmetic(True)

        self.body = QGraphicsRectItem()
        self.body.setBrush(QBrush(color))
        self.body.setPen(pen)
        self.body.setZValue(5)
        scene.addItem(self.body)

        # heading 은 body 의 자식이라 body 의 회전을 그대로 물려받는다
        self.heading = QGraphicsPolygonItem(self.body)
        self.heading.setBrush(QBrush(color))
        self.heading.setPen(pen)
        self.heading.setZValue(6)

        self.label = _OutlinedLabel()
        f = self.label.font()
        f.setPointSizeF(8.0)
        f.setBold(True)
        self.label.setFont(f)
        self.label.setZValue(7)
        scene.addItem(self.label)

        path_pen = QPen(color)
        path_pen.setWidthF(1.6)
        path_pen.setCosmetic(True)
        self.path_item = QGraphicsPathItem()
        self.path_item.setPen(path_pen)
        self.path_item.setZValue(3)
        scene.addItem(self.path_item)

        # 이 로봇이 감지한 동적 장애물 — 개수가 매번 바뀌므로 필요한 만큼 만들고
        # 남는 건 재사용한다 (update_obstacles 에서 관리)
        self.obstacle_items: list[QGraphicsRectItem] = []

    def remove(self, scene: QGraphicsScene) -> None:
        for item in (self.body, self.label, self.path_item, *self.obstacle_items):
            scene.removeItem(item)

    def update_pose(self, meta, view: RobotView, label_text: str, alpha: float) -> None:
        length_px = max(view.length, 1e-3) / meta.resolution
        width_px = max(view.width, 1e-3) / meta.resolution

        self.body.setRect(-length_px / 2, -width_px / 2, length_px, width_px)
        tip = length_px * 0.5
        back = -length_px * 0.1
        half_w = width_px * 0.32
        self.heading.setPolygon(QPolygonF([
            QPointF(tip, 0), QPointF(back, -half_w), QPointF(back, half_w),
        ]))

        pos = world_to_scene(meta, view.x, view.y)
        self.body.setPos(pos)
        # world 도 y-down 이라 atan2(dy,dx) 가 Qt 의 "양수=시계방향" 규약과 그대로 맞는다
        self.body.setRotation(math.degrees(view.theta))
        self.body.setOpacity(alpha)

        self.label.setText(label_text)
        br = self.label.boundingRect()
        self.label.setPos(pos.x() - br.width() / 2, pos.y() - width_px * 0.9 - br.height())
        self.label.setOpacity(alpha)

    def update_path(self, meta, view: RobotView, show: bool, alpha: float) -> None:
        if not show or len(view.planned_path) < 1:
            self.path_item.setPath(QPainterPath())
            return
        points = [world_to_scene(meta, view.x, view.y)]
        points += [world_to_scene(meta, x, y) for x, y in view.planned_path]
        self.path_item.setPath(_ticked_path(points))
        self.path_item.setOpacity(alpha)

    def update_obstacles(self, scene: QGraphicsScene, meta, view: RobotView, alpha: float) -> None:
        """이 로봇이 지금 감지한 장애물만 그린다 — 다른 로봇 화면엔 안 보일 수 있다."""
        obstacles = view.detected_obstacles
        while len(self.obstacle_items) < len(obstacles):
            item = QGraphicsRectItem()
            pen = QPen(QColor("white"))
            pen.setWidthF(1.0)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setBrush(QBrush(QColor("black")))   # 검은색 단색 — 정적 장애물과 같은 색으로 통일
            item.setZValue(4)
            scene.addItem(item)
            self.obstacle_items.append(item)
        while len(self.obstacle_items) > len(obstacles):
            scene.removeItem(self.obstacle_items.pop())

        for item, (ox, oy, ow, oh) in zip(self.obstacle_items, obstacles):
            w_px, h_px = ow / meta.resolution, oh / meta.resolution
            item.setRect(-w_px / 2, -h_px / 2, w_px, h_px)
            item.setPos(world_to_scene(meta, ox, oy))
            item.setOpacity(alpha)


def _ticked_path(points: list[QPointF]) -> QPainterPath:
    """GoCart Viz 스타일 — 경로선 위에 진행방향과 수직인 짧은 눈금을 일정 간격으로 찍는다."""
    path = QPainterPath(points[0])
    for p in points[1:]:
        path.lineTo(p)

    since_tick = 0.0
    tick_len = PATH_TICK_SPACING_PX * 0.4
    for a, b in zip(points, points[1:]):
        seg = QPointF(b.x() - a.x(), b.y() - a.y())
        seg_len = math.hypot(seg.x(), seg.y())
        if seg_len < 1e-6:
            continue
        ux, uy = seg.x() / seg_len, seg.y() / seg_len
        nx, ny = -uy, ux  # 진행방향에 수직인 단위벡터
        d = PATH_TICK_SPACING_PX - since_tick
        while d < seg_len:
            cx, cy = a.x() + ux * d, a.y() + uy * d
            path.moveTo(cx - nx * tick_len / 2, cy - ny * tick_len / 2)
            path.lineTo(cx + nx * tick_len / 2, cy + ny * tick_len / 2)
            path.moveTo(cx, cy)  # 다음 lineTo 가 눈금에서 이어지지 않도록
            d += PATH_TICK_SPACING_PX
        since_tick = seg_len - (d - PATH_TICK_SPACING_PX)
    return path


class MapCanvas(QGraphicsView):
    """맵 + 로봇을 그리는 캔버스. 확대/이동은 Qt 기본 기능을 그대로 쓴다."""

    def __init__(self, map_model: MapModel, parent=None):
        super().__init__(parent)
        self.map = map_model
        self.robots: dict[str, RobotView] = {}
        self._robot_items: dict[str, _RobotItem] = {}
        self._show_grid = True
        self._show_planned_path = True
        # robot_id -> 그 로봇의 가장 최근 이벤트 한 줄. 예전엔 deque 로 모든 로봇의
        # 이벤트를 시간순으로 쌓아서, 화면에 안 남은 지 오래된 로봇의 옛날 이벤트가
        # 계속 보이는 문제가 있었다 — 이제 로봇마다 최신 값 하나만 남긴다
        self._events: dict[str, str] = {}

        self.setRenderHint(self._antialiasing_hint())
        self.setDragMode(QGraphicsView.ScrollHandDrag)   # 드래그 이동 — Qt 기본 제공
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self.setBackgroundBrush(QBrush(QColor("#0b0b0d")))
        self.setMouseTracking(True)

        self._show_zones = True
        self._zone_items: list[QGraphicsPolygonItem] = []

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._build_map_layer()
        self._build_zones()
        self._build_hud()
        self._fit_done = False

    @staticmethod
    def _antialiasing_hint():
        from PySide6.QtGui import QPainter
        return QPainter.Antialiasing

    # -- 맵 배경 ------------------------------------------------------------

    def _build_map_layer(self) -> None:
        m = self.map.metadata
        codes = self.map.classify_grid()  # 0=OCCUPIED,1=UNKNOWN,2=FREE
        rgb = np.zeros((*codes.shape, 3), dtype=np.uint8)
        for code, color in enumerate(CELL_COLORS):
            mask = codes == code
            rgb[mask, 0] = color.red()
            rgb[mask, 1] = color.green()
            rgb[mask, 2] = color.blue()
        self._classified_rgb = np.ascontiguousarray(rgb)
        self._raw_grey = np.ascontiguousarray(self.map.grid)

        self._map_item = QGraphicsPixmapItem()
        self._map_item.setZValue(0)
        self._scene.addItem(self._map_item)
        self._classified = True
        self._refresh_map_pixmap()

        self._scene.setSceneRect(0, 0, m.width_px, m.height_px)

        self._grid_item = QGraphicsPathItem()
        pen = QPen(QColor(0, 144, 255, 70))
        pen.setWidthF(0.0)
        pen.setCosmetic(True)
        self._grid_item.setPen(pen)
        self._grid_item.setZValue(1)
        self._scene.addItem(self._grid_item)
        self._build_grid_path()

        origin_pen = QPen(QColor("#ff3b30"))
        origin_pen.setWidthF(2.0)
        origin_pen.setCosmetic(True)
        cross = QPainterPath()
        r = 0.15 / m.resolution
        ox, oy = world_to_scene(m, 0.0, 0.0).x(), world_to_scene(m, 0.0, 0.0).y()
        cross.moveTo(ox - r, oy)
        cross.lineTo(ox + r, oy)
        cross.moveTo(ox, oy - r)
        cross.lineTo(ox, oy + r)
        origin_item = QGraphicsPathItem(cross)
        origin_item.setPen(origin_pen)
        origin_item.setZValue(9)
        self._scene.addItem(origin_item)

    def _build_zones(self) -> None:
        """
        `zone_meta.json` (벤더 원본, 맵 메타데이터와 같은 폴더) 을 읽어 다각형을
        그린다. 좌표는 이미 world 프레임이다 (fms_server/mapkit/bake.py 가 origin
        보정 없이 그대로 쓰는 것과 동일 — 검증됨). 파일이 없으면 조용히 건너뛴다.
        **표시 전용** — 로봇 내비게이션(장애물 회피)은 이 파일을 안 보고, FMS 가
        prohibit zone 을 grid 에 구워서 로봇에게 내려주는 별도 경로를 그대로 쓴다.
        """
        zone_path = self.map.base_dir / "zone_meta.json"
        if not zone_path.is_file():
            return
        try:
            data = json.loads(zone_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            print(f"[zone] {zone_path} 읽기 실패, 건너뜀: {exc}")
            return

        m = self.map.metadata
        for entry in data.get("zones", []):
            polygon = entry.get("polygon") or []
            if len(polygon) < 3:
                continue
            ztype = str(entry.get("type", "")).lower()
            color = ZONE_COLORS.get(ztype, ZONE_DEFAULT_COLOR)

            points = [world_to_scene(m, float(p[0]), float(p[1])) for p in polygon]
            item = QGraphicsPolygonItem(QPolygonF(points))
            fill = QColor(color)
            fill.setAlpha(90)
            item.setBrush(QBrush(fill))
            pen = QPen(color)
            pen.setWidthF(1.5)
            pen.setCosmetic(True)
            item.setPen(pen)
            item.setZValue(2)  # 맵(0)/격자(1) 위, 로봇(3+) 아래
            item.setToolTip(f"{entry.get('name', '')} ({ztype or 'unknown'})".strip())
            self._scene.addItem(item)
            self._zone_items.append(item)

        if self._zone_items:
            print(f"[zone] {zone_path.name} 에서 {len(self._zone_items)}개 zone 표시")

    def _refresh_map_pixmap(self) -> None:
        m = self.map.metadata
        if self._classified:
            arr = self._classified_rgb
            image = QImage(arr.data, m.width_px, m.height_px, 3 * m.width_px, QImage.Format_RGB888)
        else:
            arr = self._raw_grey
            image = QImage(arr.data, m.width_px, m.height_px, m.width_px, QImage.Format_Grayscale8)
        self._map_item.setPixmap(QPixmap.fromImage(image.copy()))

    def _build_grid_path(self, step_m: float = 1.0) -> None:
        m = self.map.metadata
        step_px = step_m / m.resolution
        path = QPainterPath()
        x = 0.0
        while x <= m.width_px:
            path.moveTo(x, 0)
            path.lineTo(x, m.height_px)
            x += step_px
        y = 0.0
        while y <= m.height_px:
            path.moveTo(0, y)
            path.lineTo(m.width_px, y)
            y += step_px
        self._grid_item.setPath(path)
        self._grid_item.setVisible(self._show_grid)

    # -- HUD 오버레이 (viewport 자식 위젯, scene 이 아니라 화면에 고정) -----------

    def _build_hud(self) -> None:
        def hud_label() -> QLabel:
            lbl = QLabel(self.viewport())
            lbl.setStyleSheet(
                "background-color: rgba(0,0,0,140); color: white; "
                "padding: 4px 7px; border-radius: 4px;"
            )
            lbl.setFont(QFont("Menlo, Consolas, monospace", 9))
            lbl.setTextFormat(Qt.PlainText)
            lbl.show()
            return lbl

        m = self.map.metadata
        self.title_label = hud_label()
        self.title_label.setText(
            f"{m.name}  —  {m.width_m:g} x {m.height_m:g} m  @ {m.resolution:g} m/px"
        )

        self.legend_label = hud_label()
        legend = "■ OCCUPIED   ■ UNKNOWN   ■ FREE"
        if self._zone_items:
            legend += "     ▨ PROHIBIT   ▨ SLOW   ▨ ONEWAY  (z 토글)"
        self.legend_label.setText(legend)

        self.event_label = hud_label()
        self.event_label.setText("")
        self.event_label.setWordWrap(False)

        self.info_label = hud_label()
        self.info_label.setText("no robots")

        self.coord_label = hud_label()
        self.coord_label.setText("")

        self._layout_hud()

    def _layout_hud(self) -> None:
        # setText() 는 크기를 자동으로 안 늘려준다 — 배치 전에 매번 다시 재야 한다
        for lbl in (self.title_label, self.legend_label, self.event_label,
                    self.info_label, self.coord_label):
            lbl.adjustSize()

        vp = self.viewport().rect()
        pad = 8
        self.title_label.move(pad, pad)
        self.legend_label.move(pad, vp.height() - self.legend_label.height() - pad)
        self.event_label.move(pad, self.title_label.geometry().bottom() + 4)
        self.info_label.move(vp.width() - self.info_label.width() - pad, pad)
        self.coord_label.move(
            vp.width() - self.coord_label.width() - pad,
            vp.height() - self.coord_label.height() - pad,
        )

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._layout_hud()
        if not self._fit_done:
            self.reset_view()
            self._fit_done = True

    # -- 로봇 데이터 갱신 (표시용 입력, fleet_monitor 가 호출) ------------------

    def update_robot(
        self, robot_id: str, x: float, y: float, theta: float = 0.0, *,
        velocity: float = 0.0, width: float = 0.5, length: float = 0.7,
        state: str = "?", battery: Optional[float] = None,
        order_id: Optional[str] = None, remaining_nodes: int = 0,
        errors: Optional[list[str]] = None,
        planned_path: Optional[list[tuple[float, float]]] = None,
        detected_obstacles: Optional[list[tuple[float, float, float, float]]] = None,
        now: float = 0.0,
    ) -> None:
        view = self.robots.get(robot_id)
        if view is None:
            view = RobotView(robot_id=robot_id)
            self.robots[robot_id] = view
        view.x, view.y, view.theta = x, y, theta
        view.velocity = velocity
        view.width, view.length = width, length
        view.state = state
        view.battery = battery
        view.order_id = order_id
        view.remaining_nodes = remaining_nodes
        view.errors = errors or []
        view.planned_path = list(planned_path) if planned_path else []
        view.detected_obstacles = list(detected_obstacles) if detected_obstacles else []
        view.last_seen = now
        view.has_pose = True

    def set_connection(self, robot_id: str, connection: str, now: float = 0.0) -> None:
        view = self.robots.get(robot_id)
        if view is None:
            view = RobotView(robot_id=robot_id, last_seen=now)
            self.robots[robot_id] = view
        view.connection = connection

    def remove_robot(self, robot_id: str) -> None:
        self.robots.pop(robot_id, None)
        self._events.pop(robot_id, None)
        item = self._robot_items.pop(robot_id, None)
        if item is not None:
            item.remove(self._scene)
        self.event_label.setText("\n".join(self._events[k] for k in sorted(self._events)))
        self._layout_hud()

    def append_event(self, robot_id: str, text: str) -> None:
        """등록된 로봇마다 가장 최근 이벤트 한 줄만 남긴다 — 지나간 이벤트나
        더 이상 안 보이는 로봇의 옛 기록이 패널에 계속 남지 않게 한다."""
        self._events[robot_id] = text
        lines = [self._events[k] for k in sorted(self._events)][-EVENT_LOG_MAX_LINES:]
        self.event_label.setText("\n".join(lines))
        self._layout_hud()

    def robot_color(self, robot_id: str) -> QColor:
        ordered = sorted(self.robots)
        idx = ordered.index(robot_id) if robot_id in ordered else 0
        return QColor(ROBOT_PALETTE[idx % len(ROBOT_PALETTE)])

    # -- 렌더링 갱신 ----------------------------------------------------------

    def refresh(self, now: float = 0.0) -> None:
        m = self.map.metadata
        live = sum(1 for v in self.robots.values() if v.has_pose)
        lines = [f"ROBOTS: {live} reporting / {len(self.robots)} known", ""]

        seen = set()
        for robot_id, view in sorted(self.robots.items()):
            if not view.has_pose:
                lines.append(f"- {robot_id}  (no state)")
                lines.append(f"    conn {view.connection}")
                lines.append("")
                continue
            seen.add(robot_id)
            color = self.robot_color(robot_id)
            stale = view.is_stale(now)
            alpha = 0.3 if stale else 0.95
            label_text = robot_id.replace("AMR-", "")

            item = self._robot_items.get(robot_id)
            if item is None:
                item = _RobotItem(self._scene, color)
                self._robot_items[robot_id] = item
            item.update_pose(m, view, label_text, alpha)
            item.update_path(m, view, self._show_planned_path, 0.25 if stale else 0.75)
            item.update_obstacles(self._scene, m, view, 0.25 if stale else 0.85)

            mark = "x" if stale else "*"
            battery = f"{view.battery:5.1f}%" if view.battery is not None else "    ?"
            lines.append(f"{mark} {robot_id}  {view.state}")
            lines.append(f"    xy   {view.x:7.2f}, {view.y:6.2f}")
            lines.append(f"    th   {view.theta:7.3f} rad")
            lines.append(f"    v    {view.velocity:7.2f} m/s")
            lines.append(f"    bat  {battery}")
            if view.order_id:
                lines.append(f"    ord  {view.order_id} ({view.remaining_nodes} left)")
            if view.connection not in ("ONLINE", "?"):
                lines.append(f"    conn {view.connection}")
            if view.errors:
                lines.append(f"    ERR  {', '.join(view.errors)}")
            if stale:
                lines.append(f"    (no report for {now - view.last_seen:.0f}s)")
            lines.append("")

        for robot_id in [r for r in self._robot_items if r not in seen and r not in self.robots]:
            self._robot_items.pop(robot_id).remove(self._scene)

        self.info_label.setText("\n".join(lines) if self.robots else "no robots")
        self._layout_hud()

    # -- 조작: 확대/이동/토글 --------------------------------------------------

    def wheelEvent(self, event) -> None:
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

    def reset_view(self) -> None:
        self.fitInView(QRectF(0, 0, self.map.metadata.width_px, self.map.metadata.height_px),
                        Qt.KeepAspectRatio)

    def mouseMoveEvent(self, event) -> None:
        super().mouseMoveEvent(event)
        scene_pos = self.mapToScene(event.position().toPoint())
        x, y = scene_to_world(self.map.metadata, scene_pos.x(), scene_pos.y())
        if self.map.in_bounds(x, y):
            cell = self.map.cell_at(x, y)
            mark = "drivable" if cell.drivable else "blocked"
            self.coord_label.setText(f"x={x:7.2f} m  y={y:7.2f} m   {cell.value} ({mark})")
        else:
            self.coord_label.setText(f"x={x:7.2f} m  y={y:7.2f} m   [out of bounds]")
        self.coord_label.adjustSize()
        self._layout_hud()

    def keyPressEvent(self, event) -> None:
        key = event.text().lower()
        if key == "g":
            self._show_grid = not self._show_grid
            self._grid_item.setVisible(self._show_grid)
        elif key == "c":
            self._classified = not self._classified
            self._refresh_map_pixmap()
        elif key == "t":
            self._show_planned_path = not self._show_planned_path
        elif key == "z":
            self._show_zones = not self._show_zones
            for item in self._zone_items:
                item.setVisible(self._show_zones)
        elif key == "r":
            self.reset_view()
        else:
            super().keyPressEvent(event)


class FleetViewWindow(QMainWindow):
    """맵 전체를 채우는 창. 정보는 전부 그 위 오버레이다."""

    def __init__(self, map_model: MapModel, title: str = "Fleet Viz"):
        super().__init__()
        self.setWindowTitle(title)
        if ICON_PATH.exists():
            self.setWindowIcon(QIcon(str(ICON_PATH)))
        self.canvas = MapCanvas(map_model)
        self.setCentralWidget(self.canvas)
        self.resize(1280, 800)

    # MapCanvas 로 위임 (fleet_monitor.py 가 부르는 인터페이스 그대로)
    def update_robot(self, *a, **kw) -> None:
        self.canvas.update_robot(*a, **kw)

    def set_connection(self, *a, **kw) -> None:
        self.canvas.set_connection(*a, **kw)

    def remove_robot(self, *a, **kw) -> None:
        self.canvas.remove_robot(*a, **kw)

    def append_event(self, robot_id: str, text: str) -> None:
        self.canvas.append_event(robot_id, text)

    def refresh(self, now: float = 0.0) -> None:
        self.canvas.refresh(now)
