from itertools import combinations
from typing import List, Tuple

import networkx as nx
import numpy as np
from fastdtw import fastdtw
from lanelet2.core import Lanelet
from scipy.spatial.distance import euclidean
from shapely.geometry import LineString
from shapely.ops import substring

from lanelet2anchors.anchor_tools.anchor2linestring import _anchor2linestring


def sort_anchors(anchors: List[Tuple[Lanelet, ...]], distance_method: str):
    centerlines = {
        anchor: _anchor2linestring(anchor, "centerline") for anchor in anchors
    }

    G = nx.Graph()

    for anchor in anchors:
        G.add_node(anchor)

    for (a1, l1), (a2, l2) in combinations(centerlines.items(), 2):
        G.add_edge(a1, a2, weight=_line_distance(l1, l2, distance_method))

    sorted_anchors = []

    while len(G) > 1:
        node_weights = {
            node: sum(edge[2] for edge in G.edges(node, data="weight"))
            for node in G.nodes()
        }

        node_to_remove = min(node_weights, key=node_weights.get)  # type: ignore

        G.remove_node(node_to_remove)
        sorted_anchors.append(node_to_remove)

    sorted_anchors.extend(G.nodes)

    return list(reversed(sorted_anchors))


def _line_distance(l1: LineString, l2: LineString, method: str) -> float:
    min_length = min(l1.length, l2.length)

    l1 = substring(l1, 0, min_length)
    l2 = substring(l2, 0, min_length)

    function_map = {
        "iou": _line_iou_distance,
        "dtw": _line_dtw_distance,
        "hausdorff": _line_hausdorff_distance,
    }

    return function_map[method](l1=l1, l2=l2)


def _line_iou_distance(l1: LineString, l2: LineString) -> float:
    l1 = l1.buffer(1)
    l2 = l2.buffer(1)

    intersection = l1.intersection(l2).area
    union = l1.union(l2).area

    return 1 - intersection / union


def _line_dtw_distance(l1: LineString, l2: LineString) -> float:
    l1 = _dense_sample_line(l1)
    l2 = _dense_sample_line(l2)

    l1_data = np.array(l1.coords.xy).squeeze().transpose(1, 0)
    l2_data = np.array(l2.coords.xy).squeeze().transpose(1, 0)

    distance, _ = fastdtw(l1_data, l2_data, dist=euclidean)

    return distance


def _line_hausdorff_distance(l1: LineString, l2: LineString) -> float:
    return l1.hausdorff_distance(l2)


def _dense_sample_line(line: LineString, num_points=100) -> LineString:
    return LineString(
        [line.interpolate(i, normalized=True) for i in np.linspace(0, 1, num_points)]
    )
