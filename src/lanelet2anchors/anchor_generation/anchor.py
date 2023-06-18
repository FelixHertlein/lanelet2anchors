from dataclasses import dataclass
from typing import Tuple

from lanelet2.core import Lanelet


@dataclass
class Anchor:
    """Anchor path defined as a sequence of lanelets.

    Args:
        lanelets (Tuple[Lanelet, ...]): Sequence of lanelets.
    """

    lanelets: Tuple[Lanelet, ...]

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Anchor):
            return False
        return self.lanelets == other.lanelets

    def __hash__(self) -> int:
        return hash(self.lanelets)
