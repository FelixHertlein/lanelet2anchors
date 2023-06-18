from typing import List, Tuple

import networkx as nx
from lanelet2.core import Lanelet
from lanelet2.routing import RelationType, RoutingGraph

from ..anchor_tools.anchor2linestring import _anchor2linestring


def discover_anchors(
    routing_graph: RoutingGraph,
    start_lanelet: Lanelet,
    max_length: float,
) -> List[Tuple[Lanelet, ...]]:
    G = nx.DiGraph()
    G.add_node(0, lanelet=start_lanelet)
    _traverse_routing_graph(G, routing_graph, start_lanelet, 0, max_length)

    anchors = _graph2anchors(G)

    return anchors


def _traverse_routing_graph(
    G: nx.DiGraph,
    routing_graph: RoutingGraph,
    current_lanelet: Lanelet,
    current_id: int,
    max_length: float,
) -> nx.DiGraph:
    if _determine_anchor_length(G, current_id) > max_length:
        return

    following_lanelets = routing_graph.followingRelations(
        current_lanelet, withLaneChanges=True
    )

    for relation in following_lanelets:
        new_id = len(G.nodes)

        G.add_node(new_id, lanelet=relation.lanelet)
        G.add_edge(current_id, new_id, relation_type=relation.relationType)

        if _containes_bidirectional_lane_changes(G, new_id):
            G.remove_node(new_id)
            continue

        _traverse_routing_graph(G, routing_graph, relation.lanelet, new_id, max_length)


def _determine_anchor_length(G: nx.DiGraph, current_id: int) -> float:
    ancestors_graph = _ancestors_graph(G, current_id)
    ancestors_list = list(nx.topological_sort(ancestors_graph))

    anchor = tuple(G.nodes[ancestor]["lanelet"] for ancestor in ancestors_list)
    anchor_short = anchor[1:]

    return _anchor2linestring(anchor_short, "centerline").length


def _graph2anchors(G: nx.DiGraph) -> List[Tuple[Lanelet, ...]]:
    leafs = [node for node in G.nodes if G.out_degree(node) == 0]

    anchors: List[Tuple[Lanelet, ...]] = []
    for leaf in leafs:
        ancestors_graph = _ancestors_graph(G, leaf)
        ancestors_list = list(nx.topological_sort(ancestors_graph))

        lanelets: List[Lanelet] = [
            G.nodes[ancestor]["lanelet"] for ancestor in ancestors_list
        ]
        anchors.append(tuple(lanelets))

    return anchors


def _containes_bidirectional_lane_changes(G: nx.DiGraph, current_id: int) -> bool:
    ancestors_graph = _ancestors_graph(G, current_id)

    relation_types = set(
        edge[2] for edge in ancestors_graph.edges(data="relation_type")
    )

    expected_types = {RelationType.Successor, RelationType.Left, RelationType.Right}
    assert len(relation_types - expected_types) == 0

    return RelationType.Left in relation_types and RelationType.Right in relation_types


def _ancestors_graph(G: nx.DiGraph, current_id: int) -> nx.DiGraph:
    ancestors_set = nx.ancestors(G, current_id).union(set([current_id]))
    return G.subgraph(ancestors_set)
