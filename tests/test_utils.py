import pytest

from utils.interpolating_map import InterpolatingMap


@pytest.fixture
def table():
    m = InterpolatingMap()
    m.insert(1.0, 10.0)
    m.insert(3.0, 30.0)
    return m


def test_interpolates_between_points(table):
    assert table.get(2.0) == pytest.approx(20.0)


def test_clamps_outside_range(table):
    assert table.get(0.0) == 10.0
    assert table.get(99.0) == 30.0


def test_empty_map_returns_zero():
    assert InterpolatingMap().get(5.0) == 0.0
