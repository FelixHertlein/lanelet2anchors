import numpy as np
from lanelet2.core import LaneletMap
from shapely.geometry import LineString


def interpolate_lanelet(
    lanelet_map: LaneletMap,
    lanelet_id: int,
    ratio: float,
    num_points: int = 100,
) -> LineString:
    lanelet = lanelet_map.laneletLayer[lanelet_id]

    left_border = np.array([(point.x, point.y) for point in lanelet.leftBound])
    right_border = np.array([(point.x, point.y) for point in lanelet.rightBound])

    left_linestring = LineString(left_border)
    right_linestring = LineString(right_border)

    left_border_interp = np.array(
        [
            left_linestring.interpolate(i, normalized=True).xy
            for i in np.linspace(0, 1, num_points)
        ]
    ).squeeze()
    right_border_interp = np.array(
        [
            right_linestring.interpolate(i, normalized=True).xy
            for i in np.linspace(0, 1, num_points)
        ]
    ).squeeze()

    interp_data = left_border_interp * ratio + right_border_interp * (1 - ratio)
    return LineString(interp_data)
