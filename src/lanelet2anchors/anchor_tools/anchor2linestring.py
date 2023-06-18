from itertools import groupby, zip_longest
from typing import Dict, List, Optional, Tuple

import numpy as np
import scipy
from lanelet2.core import Lanelet, LaneletMap
from shapely.geometry import LineString, Point
from shapely.ops import substring

from ..anchor_generation.anchor import Anchor

SMOOTH_INTERPOLATOR = scipy.interpolate.PchipInterpolator(
    np.array([-1, 0, 1, 2]), np.array([0, 0, 1, 1]), axis=0, extrapolate=None
)

LINE_TYPE_MAP = {"left": "leftBound", "right": "rightBound", "center": "centerline"}


def anchor2linestring(
    anchor: Anchor, line_type: str, trim_point: Optional[Point] = None
) -> LineString:
    """Converts a given anchor to LineStrings based on the anchor lanelets. Depending on the parameter 'line_type', the function generates Linestrings from the anchor boundaries or the centerline. In the case of adjacent lanelets within the anchor, the lines are smoothly interpolated.

    Args:
        anchor (Anchor): Lanelet anchor to convert.
        line_type (str): Type of line to convert. Valid options are 'left', 'right' and 'center'.
        trim_point (Optional[Point], optional): _description_. Defaults to None.

    Returns:
        LineString: LineString representing the boundary or centerline of an anchor.
    """
    assert line_type in [
        "left",
        "right",
        "center",
    ], "Invalid line type encountered! Select one of 'left', 'right', 'center'!"

    return _anchor2linestring(
        lanelets=anchor.lanelets,
        lanlet2_line_type=LINE_TYPE_MAP[line_type],
        trim_point=trim_point,
    )


def _anchor2linestring(
    lanelets: Tuple[Lanelet, ...],
    lanlet2_line_type: str,
    trim_point: Optional[Point] = None,
) -> LineString:
    # gather all lines
    lines = [
        LineString([Point(pos.x, pos.y) for pos in getattr(lanelet, lanlet2_line_type)])
        for lanelet in lanelets
    ]

    # combine lines to single linestring
    combined_line = _combine_linestrings(lines)

    # cut off linesting
    if trim_point:
        combined_line = _trim_linestring_by_point(combined_line, trim_point)

    return combined_line


def _trim_linestring_by_point(line: LineString, trim_point: Point) -> LineString:
    progress = line.project(trim_point, normalized=True)

    return substring(line, progress, 1, normalized=True)


def _combine_linestrings(
    lines: List[LineString],
    close_points_threshold: float = 0.1,
    simplify_threshold: float = 0.01,
) -> LineString:
    def is_lane_change(pair):
        l1, l2 = pair
        end_point = Point(l1.coords[-1])
        start_point = Point(l2.coords[0]) if l2 else end_point
        return start_point.distance(end_point) > close_points_threshold

    # list all consecutive pairs
    all_pairs = list(zip_longest(lines, lines[1:]))

    # compress multi-lane changes to single change
    filtered_pairs = []
    for is_change, group_pairs in groupby(all_pairs, key=is_lane_change):
        group_pairs = list(group_pairs)  # type: ignore

        if is_change:
            filtered_pairs.append((group_pairs[0][0], group_pairs[-1][1]))  # type: ignore
        else:
            filtered_pairs.extend(group_pairs)

    # construct the final linestring
    result = LineString([])

    line_pairs = iter(filtered_pairs)

    for l1, l2 in line_pairs:
        if is_lane_change((l1, l2)):
            l_blend = _blend_linestrings_smoothely(l1, l2)
            result = _concat_linestrings(result, l_blend)

            next(line_pairs, None)  # skipp the next iteration
        else:
            result = _concat_linestrings(result, l1)

    return result.simplify(simplify_threshold)


def _concat_linestrings(line1: LineString, line2: LineString) -> LineString:
    return LineString(list(line1.coords) + list(line2.coords))


def _blend_linestrings_smoothely(
    line1: LineString, line2: LineString, num_points: int = 100
) -> LineString:
    return LineString(
        [
            np.array(line1.interpolate(alpha, normalized=True).coords).squeeze(0)
            * (1 - SMOOTH_INTERPOLATOR(alpha))
            + np.array(line2.interpolate(alpha, normalized=True).coords).squeeze(0)
            * SMOOTH_INTERPOLATOR(alpha)
            for alpha in np.linspace(0, 1, num_points)
        ]
    )
