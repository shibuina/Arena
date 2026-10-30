"""Open fields: square obstacles in a walled box."""

import functools
import random

import pytest
import shapely
from arena_simulation_setup.generative.world.open import WorldGeneratorOpen
from arena_simulation_setup.tree.World import Level


def build(seed: int = 0, **config: float) -> tuple[WorldGeneratorOpen, Level]:
    generator = WorldGeneratorOpen(config, random.Random(seed))
    return generator, generator.compute().all_levels[0]


def fill(level: Level, area: float) -> float:
    """Occupied share of the box. The walls are the rings of the free space, so a point is free inside an odd number of them."""
    free = functools.reduce(shapely.symmetric_difference, (shapely.Polygon([(p.x, p.y) for p in wall.path]) for wall in level.walls))
    return 1 - free.area / area


def test_no_obstacles_is_the_bare_box():
    generator, level = build(width=12.0, height=8.0, obstacles=0)
    assert generator.diagnostics.islands == 0
    assert len(level.zones) == 1 and len(level.walls) == 1
    assert fill(level, 96.0) == pytest.approx(0.0)


def test_an_obstacle_inside_is_walled_and_an_island():
    generator, level = next(built for built in (build(seed, width=20.0, height=20.0, obstacles=1, obstacle_size=4.0) for seed in range(20)) if built[0].diagnostics.islands)
    assert len(level.walls) == 2
    assert fill(level, 400.0) == pytest.approx(16.0 / 400.0)


def test_one_zone_however_many_obstacles():
    generator, level = build(obstacles=300, obstacle_size=0.5)
    assert len(level.zones) == 1 and generator.diagnostics.islands > 100


def test_fill_grows_with_the_count():
    fills = []
    for n in (5, 20, 60):
        fills.append(fill(build(1, obstacles=n)[1], 400.0))
    assert 0 < fills[0] < fills[1] < fills[2] < 0.5


def test_a_seed_repeats():
    assert build(4, obstacles=25, size_spread=0.5)[1].walls == build(4, obstacles=25, size_spread=0.5)[1].walls
