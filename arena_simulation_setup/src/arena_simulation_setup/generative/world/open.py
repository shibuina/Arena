"""Open fields of square obstacles."""

import pydantic
import shapely

from arena_simulation_setup.tree.World import Level, World, Zone

from . import PedestrianConfiguration, WithPedestrians
from .layout import Diagnostics
from .utils import to_corners, to_walls, world_of


class WorldGeneratorOpen(WithPedestrians):
    """A walled box with square obstacles at random positions, merged where they overlap."""

    class Configuration(PedestrianConfiguration):
        width: float = pydantic.Field(20.0, gt=0)
        height: float = pydantic.Field(20.0, gt=0)
        obstacles: int = pydantic.Field(15, ge=0)
        obstacle_size: float = pydantic.Field(1.0, gt=0)
        size_spread: float = pydantic.Field(0.0, ge=0, lt=1)

    config: Configuration

    def configure(self, configuration: dict):
        self.config = self.Configuration.model_validate(configuration)
        self.warnings = []
        self.diagnostics = None

    def compute(self) -> World:
        c = self.config
        box = shapely.box(0, 0, c.width, c.height)
        blocks = []
        for _ in range(c.obstacles):
            side = c.obstacle_size * self.rng.uniform(1 - c.size_spread, 1 + c.size_spread)
            x, y = self.rng.uniform(0, c.width), self.rng.uniform(0, c.height)
            blocks.append(shapely.box(x - side / 2, y - side / 2, x + side / 2, y + side / 2))
        free = box.difference(shapely.union_all(blocks)) if blocks else box
        parts = [part for part in shapely.get_parts(free) if isinstance(part, shapely.Polygon) and not part.is_empty]
        fill = 1 - sum(part.area for part in parts) / box.area
        level = Level(zones=[Zone(name='open', polygon=to_corners(box))], walls=to_walls(shapely.MultiPolygon(parts)))
        self.diagnostics = Diagnostics(components=len(parts), islands=sum(len(part.interiors) for part in parts), zones=1, extent=(c.width, c.height))
        return world_of(level, {'open': f'open field, {c.obstacles} obstacles, {fill:.0%} filled'})
