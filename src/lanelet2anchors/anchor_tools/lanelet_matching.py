from collections import Counter
from dataclasses import dataclass
from typing import Dict, List, Union

import lanelet2
import matplotlib.pyplot as plt
import numpy as np
from lanelet2.core import BasicPoint2d, Lanelet, LaneletMap
from lanelet2.matching import (
    ConstLaneletMatch,
    Object2d,
    ObjectWithCovariance2d,
    Pose2d,
    PositionCovariance2d,
    getDeterministicMatches,
    getProbabilisticMatches,
)
from scipy.spatial.transform import Rotation
from shapely.geometry import LineString, Point, Polygon

from ..anchor_generation.anchor import Anchor

EPS = np.finfo(float).eps


@dataclass
class LaneletMatchProb:
    """Lanelet match with its assigned probability.

    Args:
        lanelet_match (lanelet2.core.ConstLaneletMatch): Lanelet match.
        probability (float): Probability that ego vehicle is on specified lanelet.
    """

    lanelet_match: ConstLaneletMatch
    probability: float

    @property
    def lanelet(self) -> Lanelet:
        return self.lanelet_match.lanelet


@dataclass
class LaneletAnchorMatches:
    """Matched lanelet of ego vehicle with corresponding possible anchor paths. All anchors as well as DMAP-based filtering is available.

    Args:
        lanelet_match (LaneletMatchProb): Matched lanelet with assigned probability.
        anchors (List[Anchor]): All available anchor paths for the given lanelet.
        selection (List[bool]): Mask of filtered anchor paths. True for within selection, false otherwise.
    """

    lanelet_match: LaneletMatchProb
    anchors: List[Anchor]
    selection: List[bool]

    @property
    def selected_anchors(self) -> List[Anchor]:
        return [
            self.anchors[i] for i in range(len(self.anchors)) if self.selection[i] == 1
        ]


@dataclass
class LaneletMatchingConfig:
    """Parameter for deterministic lanelet matching.

    Args:
        max_dist_to_lanelet (np.double): Maximum distance to lanelet. Defaults to 0.5.
    """

    max_dist_to_lanelet: np.double = np.double(0.5)


@dataclass
class LaneletMatchingProbConfig(LaneletMatchingConfig):
    """Parameters for probabilistic lanelet matching.

    Args:
        x_var (float): Variance of position of vehicle in x-direction. Defaults to 0.5.
        y_var (float): Variance of position of vehicle in y-direction. Defaults to 0.5.
        covar_position (float): Covariance of the position in m. Defaults to 0.0.
        covar_orientation (List[float]): Covariance of the orientation in degree. Defaults to 5.0.
    """

    x_var: float = 0.5
    y_var: float = 0.5
    covar_position: float = 0.0
    covar_orientation: float = 5.0

    @property
    def position_covariance2d(self) -> PositionCovariance2d:
        return PositionCovariance2d(self.x_var, self.y_var, self.covar_position)

    @property
    def kappa(self) -> np.double:
        return np.double(1.0 / (self.covar_orientation / 180.0 * np.pi))


@dataclass
class VehiclePose:
    """Describes the position and orientation of a vehicle.

    Args:
        x (float): Position of vehicle in x-direction.
        y (float): Position of vehicle in y-direction.
        psi (float): Orientation of vehicle as angle.
        bbox (List[float]): Oriented bounding box of vehicle [top left, bottom left, bottom right, top right].
        length (float): Length of vehicle.
        width (float): Width of vehicle.
    """

    x: float
    y: float
    psi: float
    bbox: List[float]
    length: float
    width: float

    def as_pose2d(self) -> Pose2d:
        return Pose2d(self.x, self.y, self.psi)

    def bbox_as_basic_points2d(self) -> List[BasicPoint2d]:
        return [BasicPoint2d(x_, y_) for x_, y_ in self.bbox]

    def bbox_as_shapely_polygon(self) -> Polygon:
        return Polygon(self.bbox)

    def as_object2d(self) -> Object2d:
        return Object2d(
            1,
            self.as_pose2d(),
            self.bbox_as_basic_points2d(),
        )

    def as_object2d_with_covariance(
        self, matching_config: LaneletMatchingConfig
    ) -> ObjectWithCovariance2d:
        return ObjectWithCovariance2d(
            1,
            self.as_pose2d(),
            self.bbox_as_basic_points2d(),
            matching_config.position_covariance2d,
            matching_config.kappa,
        )

    @property
    def position(self) -> List[float]:
        return [self.x, self.y]

    @classmethod
    def from_nusc(
        cls,
        x: float,
        y: float,
        rotation_quat: np.ndarray,
        vehicle_width: float,
        vehicle_length: float,
    ) -> "VehiclePose":
        bbox_init = np.array(
            [
                [-vehicle_length / 2, vehicle_width / 2, 0],
                [-vehicle_length / 2, -vehicle_width / 2, 0],
                [vehicle_length / 2, -vehicle_width / 2, 0],
                [vehicle_length / 2, vehicle_width / 2, 0],
            ]
        )
        r = Rotation.from_quat(rotation_quat).inv()
        psi = r.as_rotvec()[0]
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)
        R = np.array([[c_psi, -s_psi], [s_psi, c_psi]])

        bbox = np.dot(R, bbox_init[:, :2].T).T
        bbox[:, 0] += x
        bbox[:, 1] += y
        vehicle_pose = cls(
            x=x, y=y, psi=psi, bbox=bbox, length=vehicle_length, width=vehicle_width
        )
        return vehicle_pose


def _convert_distances_to_probabilities(distances: np.ndarray) -> np.ndarray:
    distances = distances + EPS
    closeness = 1 - (distances - np.min(distances)) / distances
    closeness[closeness < 0.95] = 0
    probs = closeness / np.sum(closeness)
    return probs
