from pathlib import Path
from typing import Dict, List, Union

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import transforms
from nuscenes.map_expansion.map_api import NuScenesMap
from PIL import Image
from shapely.geometry import GeometryCollection, LineString, Point, Polygon

from ..anchor_generation.anchor import Anchor
from ..anchor_tools.anchor2polygon import anchor2polygon
from ..anchor_tools.lanelet_matching import LaneletAnchorMatches, VehiclePose
from .anchor2linestring import anchor2linestring

ROOT = Path(__file__).parent.parent.parent.parent


def plot_trajectory_and_anchors(
    gt_trajectory: LineString,
    anchors_info: List[LaneletAnchorMatches],
    vehicle_pose: VehiclePose,
    nusc_map: NuScenesMap,
    vis_type: str = "all_dmap",
):
    """Visualize vehicle, GT trajectory and anchor paths.
    NOTE: This is only supported for nuScenes and when installing the DEV version: `pip install lanelet2anchors[dev]`

    Args:
        gt_trajectory (LineString): ground truth trajector of vehicle
        anchors_info (List[LaneletAnchorMatches]): List of anchors per lanelet match
        vehicle_pose (VehiclePose): vehicle pose
        nusc_map (NuScenesMap): nuScenes map
        vis_type (str, optional): Type of visualization. Defaults to "all_dmap".

    Returns:
        _type_: _description_
    """
    bbox_car = vehicle_pose.bbox_as_shapely_polygon()
    #  Get all anchors
    all_anchors = sum([match.anchors for match in anchors_info], [])
    all_anchor_linestrings = [
        anchor2linestring(a, "center", bbox_car.centroid) for a in all_anchors
    ]
    # Get selected anchors
    selected_anchors = sum([match.selected_anchors for match in anchors_info], [])
    selected_anchor_linestrings = [
        anchor2linestring(a, "center", bbox_car.centroid) for a in selected_anchors
    ]
    lanelets = [
        {
            "poly": anchor2polygon(Anchor([lanelet_anchors.lanelet_match.lanelet])),
            "prob": lanelet_anchors.lanelet_match.probability,
        }
        for lanelet_anchors in anchors_info
    ]
    linestrings = all_anchor_linestrings + [gt_trajectory]
    obj = GeometryCollection(linestrings + [bbox_car])

    fig, ax = _get_nusc_patch_within_bounds(
        nusc_map, render_bounds=_compute_render_bounds(obj)
    )

    if vis_type == "map":
        pass
    if vis_type in [
        "agent",
        "matches",
        "all_anchors",
        "dmap_anchors",
        "gt_dmap",
        "all_dmap",
    ]:
        ax.fill(*bbox_car.exterior.xy, color="red", linewidth=5)
        # img, extent = _get_rotated_vehicle_visualization(vehicle_pose)
        # ax.imshow(img, extent=extent, alpha=1.0, zorder=10)
    if vis_type in ["matches", "all_dmap"]:
        for lanelet_info in lanelets:
            ax.plot(*lanelet_info["poly"].exterior.xy, label=f"Start lanelet")
            ax.text(
                lanelet_info["poly"].centroid.x,
                lanelet_info["poly"].centroid.y,
                f"{round(lanelet_info['prob'] * 100)}%",
            )
    if vis_type == "all_anchors":
        for idx, linestring in enumerate(all_anchor_linestrings):
            ax.plot(*linestring.xy, linewidth=3, alpha=0.5, label=f"Anchor {idx}")
    if vis_type in ["dmap_anchors", "gt_dmap", "all_dmap"]:
        for idx, linestring in enumerate(selected_anchor_linestrings):
            ax.plot(*linestring.xy, linewidth=3, alpha=0.5, label=f"Anchor {idx}")
    if vis_type in ["gt_dmap", "all_dmap"]:
        ax.plot(*gt_trajectory.xy, color="blue", linewidth=4)
    return fig, ax


def _compute_render_bounds(obj):
    bounds = [i for i in obj.bounds]
    delta_x = bounds[2] - bounds[0]
    delta_y = bounds[3] - bounds[1]
    diff = abs(delta_x - delta_y)
    if delta_y > delta_x:
        bounds[0] -= diff / 2
        bounds[2] += diff / 2
    else:
        bounds[1] -= diff / 2
        bounds[3] += diff / 2
    assert abs((bounds[2] - bounds[0]) - (bounds[3] - bounds[1])) < 10**-3
    return bounds


def _get_nusc_patch_within_bounds(nusc_map: NuScenesMap, render_bounds: List[float]):
    fig, ax = nusc_map.render_map_patch(
        render_bounds,
        figsize=(10, 10),
        layer_names=["road_segment", "drivable_area"],
        render_egoposes_range=False,
        render_legend=False,
    )
    ax.grid(False)
    ax.set_xticks([])
    ax.set_yticks([])
    return fig, ax


def _get_rotated_vehicle_visualization(vehicle_pose: VehiclePose):
    # prepare iamge
    img = Image.open(ROOT / "misc/car-top-view-icon.png")

    factor = 1.3
    width = vehicle_pose.width * factor
    height = vehicle_pose.length * factor
    angle = np.rad2deg(vehicle_pose.psi) + 90

    aspect_ratio = width / height
    if height > width:
        shape = int(img.height * aspect_ratio), img.height
    else:
        shape = img.width, int(img.width / aspect_ratio)

    img_quality = img.copy()
    img_quality = img_quality.resize(shape)
    img_quality = img_quality.rotate(angle, expand=True)

    # caclculate frame
    minx, miny, maxx, maxy = vehicle_pose.bbox_as_shapely_polygon().bounds
    return img_quality, [minx, maxx, miny, maxy]
