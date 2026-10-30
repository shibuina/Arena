"""Corridor trees: arena-rosnav's ROS 1 `rosnav` indoor maps on the layout IR."""

import itertools
import math

import pydantic
import shapely

from arena_simulation_setup.tree.World import World

from . import PedestrianConfiguration, WithPedestrians
from .layout import Area, Layout, Segment, compile_layout

Point = tuple[float, float]


class WorldGeneratorCorridor(WithPedestrians):
    """A tree of corridors grown from the center, each branch joined to the nearest node (L1)."""

    class Configuration(PedestrianConfiguration):
        width: float = pydantic.Field(30.0, gt=0)  # extent the nodes are drawn in (m)
        height: float = pydantic.Field(30.0, gt=0)
        corridor_width: float = pydantic.Field(2.0, gt=0)  # m
        width_spread: float = pydantic.Field(0.0, ge=0, lt=1)  # each branch is corridor_width x U(1 - spread, 1 + spread) wide
        branches: int = pydantic.Field(10, ge=1)  # nodes added to the root (ROS 1 `iterations`)
        bends: int = pydantic.Field(0, ge=0, le=8)  # kinks per branch, 0 keeps the right-angle elbow, more turn at 45 and 90 degrees
        wiggle: float = pydantic.Field(0.0, ge=0, le=0.5)  # sideways offset of each kink, as a share of the branch length
        diagonal: bool = True  # with bends, kinks are joined by a diagonal and a straight leg, False keeps to right angles

    config: Configuration

    def configure(self, configuration: dict):
        self.config = self.Configuration.model_validate(configuration)
        self.warnings = []
        self.diagnostics = None

    def _snap(self, value: float) -> float:
        return round(value / self.config.resolution) * self.config.resolution

    def _kinks(self, a: Point, b: Point, margin: float) -> list[Point]:
        """Up to `bends` points along a-b, each pushed across it by up to `wiggle` of its length."""
        c = self.config
        length = ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
        n = min(c.bends, int(length // (4 * c.corridor_width)) - 1)  # kinks closer than four widths zig-zag
        if n <= 0:
            return []
        nx, ny = (a[1] - b[1]) / length, (b[0] - a[0]) / length
        kinks = []
        for i in range(1, n + 1):
            t, off = i / (n + 1), self.rng.uniform(-c.wiggle, c.wiggle) * length
            x = min(max(a[0] + t * (b[0] - a[0]) + off * nx, margin), c.width - margin)
            y = min(max(a[1] + t * (b[1] - a[1]) + off * ny, margin), c.height - margin)
            kinks.append((self._snap(x), self._snap(y)))
        return kinks

    def _octilinear(self, a: Point, b: Point) -> list[Point]:
        """The turn between a diagonal and an axis leg from a to b, or between two axis legs without `diagonal`."""
        if not self.config.diagonal:
            turn = (b[0], a[1]) if self.rng.random() < 0.5 else (a[0], b[1])
            return [turn] if turn not in (a, b) else []
        dx, dy = b[0] - a[0], b[1] - a[1]
        d = min(abs(dx), abs(dy))
        turn = (a[0] + math.copysign(d, dx), a[1] + math.copysign(d, dy)) if self.rng.random() < 0.5 else (b[0] - math.copysign(d, dx), b[1] - math.copysign(d, dy))
        return [turn] if turn not in (a, b) else []

    def _winding(self, new: Point, near: Point, margin: float, width: float) -> tuple[list[Segment], list[Area]]:
        """Segments and mitered turn pieces of one octilinear branch."""
        path = [new]
        for kink in [*self._kinks(new, near, margin), near]:
            path += [p for p in (*self._octilinear(path[-1], kink), kink) if p != path[-1]]
        half = width / 2
        extent = shapely.box(0, 0, self.config.width, self.config.height)
        (px, py), (qx, qy) = path[-2], path[-1]
        run = math.dist(path[-2], path[-1])
        end = (qx + (qx - px) / run * half, qy + (qy - py) / run * half)
        segments = [Segment(a=a, b=b, width=width) for a, b in itertools.pairwise(path)]
        # the last leg runs half a width past `near`, so the junction has no notch, cut to the extent where `near` lies at
        # the margin, while the dead end at `new` stays flat
        pieces = [shapely.LineString([path[-1], end]).buffer(half, cap_style='flat')]
        for u, v, w in zip(path, path[1:-1], path[2:], strict=False):
            back = [v[i] + (u[i] - v[i]) * min(1.0, width / math.dist(u, v)) for i in (0, 1)]
            ahead = [v[i] + (w[i] - v[i]) * min(1.0, width / math.dist(v, w)) for i in (0, 1)]
            # a miter past 2 half-widths (turns sharper than 90 degrees) is beveled, and pieces at the margin are cut to the extent
            pieces.append(shapely.LineString([back, v, ahead]).buffer(half, cap_style='flat', join_style='mitre', mitre_limit=2.0))
        turns = []
        for piece in pieces:
            # a cut can leave lines or points beside the polygon, and the IR takes polygons only
            parts = [g for g in shapely.get_parts(piece.intersection(extent)) if isinstance(g, shapely.Polygon) and g.area > 0]
            if parts:
                turns.append(Area(polygon=parts[0] if len(parts) == 1 else shapely.MultiPolygon(parts)))
        return segments, turns

    def compute(self) -> World:
        c = self.config
        margin = c.corridor_width * (1 + c.width_spread) / 2
        if min(c.width, c.height) <= 2 * margin:
            raise ValueError(f'a {c.width} x {c.height} m extent leaves no room for corridors up to {2 * margin} m wide')
        nodes: list[Point] = [(self._snap(c.width / 2), self._snap(c.height / 2))]
        layout = Layout()
        for _ in range(c.branches):
            new = (self._snap(self.rng.uniform(margin, c.width - margin)), self._snap(self.rng.uniform(margin, c.height - margin)))
            near = min(nodes, key=lambda node: abs(node[0] - new[0]) + abs(node[1] - new[1]))
            if c.bends:
                for _ in range(20):  # a winding stub shorter than three widths reads as a bulge, not a corridor
                    if math.dist(new, near) >= 3 * c.corridor_width:
                        break
                    new = (self._snap(self.rng.uniform(margin, c.width - margin)), self._snap(self.rng.uniform(margin, c.height - margin)))
                    near = min(nodes, key=lambda node: abs(node[0] - new[0]) + abs(node[1] - new[1]))
                else:
                    continue
                width = c.corridor_width * self.rng.uniform(1 - c.width_spread, 1 + c.width_spread)
                segments, turns = self._winding(new, near, margin, width)
                layout.segments.extend(segments)
                layout.areas.extend(turns)
            else:
                elbow = (new[0], near[1]) if self.rng.random() < 0.5 else (near[0], new[1])
                width = c.corridor_width * self.rng.uniform(1 - c.width_spread, 1 + c.width_spread)
                layout.segments.extend(Segment(a=a, b=b, width=width) for a, b in ((new, elbow), (elbow, near)) if a != b)
                # flat caps notch at the elbow and leave the ends short, as ROS 1 carved them square
                half = width / 2
                layout.areas.extend(Area(polygon=shapely.box(x - half, y - half, x + half, y + half)) for x, y in dict.fromkeys((new, elbow, near)))
            nodes.append(new)
        if not layout.segments:
            raise ValueError(f'no branch at least {3 * c.corridor_width} m long fits a {c.width} x {c.height} m extent')
        world, self.diagnostics = compile_layout(layout)
        return world
