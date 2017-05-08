from collections import deque
from heapq import heappush, heappop
from itertools import count
import networkx as nx
from networkx.utils import generate_unique_node
import warnings as _warnings

def adapted_dijkstra_multisource(G, source, cutoff=None, target=None):
    """Uses Dijkstra's algorithm to find shortest weighted paths
    Parameters
    ----------
    G : NetworkX graph
    sources : non-empty iterable of nodes
        Starting nodes for paths. If this is just an iterable containing
        a single node, then all paths computed by this function will
        start from that node. If there are two or more nodes in this
        iterable, the computed paths may begin from any one of the start
        nodes.
    target : node label, optional
        Ending node for path. Search is halted when target is found.
    cutoff : integer or float, optional
        Depth to stop the search. Only return paths with length <= cutoff.
    Returns
    -------
    dist : dictionary
        A mapping from node to shortest distance to that node from one
        of the source nodes.
    next_node : tuple
        The first node, n, the search finds that is one level below the current node
        i.e. d_p(n) = lev - 1
    paths: dictionary
        dict to store the path list from source to each node, keyed by node.
    Notes
    -----
    The optional predecessor and path dictionaries can be accessed by
    the caller through the original pred and paths objects passed
    as arguments. No need to explicitly return pred or paths.
    """

    paths = {source: [source]}

    # define weight function
    weight = lambda u, v, data: data.get('weight', 1)

    # succ = successors
    G_succ = G.succ if G.is_directed() else G.adj

    # rename functions
    push = heappush
    pop = heappop

    dist = {}  # dictionary of final distances
    seen = {}
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []

    # current level of starting node
    cur_level = G.node[source]['dist']

    #for source in sources:
    seen[source] = 0
    push(fringe, (0, next(c), source))
    while fringe:
        (d, _, v) = pop(fringe)
        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        if G.node[v]['dist'] < cur_level:
            if cur_level == 1:
                if v in G.predecessors(v):
                    next_node = v
                    break
            else:
                next_node = v
                break

        for u, e in G_succ[v].items():
            cost = weight(v, u, e)
            if cost is None:
                continue
            vu_dist = dist[v] + cost
            if cutoff is not None:
                if vu_dist > cutoff:
                    continue
            if u in dist:
                if vu_dist < dist[u]:
                    raise ValueError('Contradictory paths found:',
                                     'negative weights?')
            elif u not in seen or vu_dist < seen[u]:
                seen[u] = vu_dist
                push(fringe, (vu_dist, next(c), u))
                if paths is not None:
                    paths[u] = paths[v] + [u]
    #print next_node
    #print type(next_node)
    return dist, next_node, paths



# def _weight_function(G, weight):
#     """Returns a function that returns the weight of an edge.
#     The returned function is specifically suitable for input to
#     functions :func:`_dijkstra` and :func:`_bellman_ford_relaxation`.
#     Parameters
#     ----------
#     G : NetworkX graph.
#     weight : string or function
#         If it is callable, `weight` itself is returned. If it is a string,
#         it is assumed to be the name of the edge attribute that represents
#         the weight of an edge. In that case, a function is returned that
#         gets the edge weight according to the specified edge attribute.
#     Returns
#     -------
#     function
#         This function returns a callable that accepts exactly three inputs:
#         a node, an node adjacent to the first one, and the edge attribute
#         dictionary for the eedge joining those nodes. That function returns
#         a number representing the weight of an edge.
#     If `G` is a multigraph, and `weight` is not callable, the
#     minimum edge weight over all parallel edges is returned. If any edge
#     does not have an attribute with key `weight`, it is assumed to
#     have weight one.
#     """
#     if callable(weight):
#         print 'callable'
#         return weight
#     # If the weight keyword argument is not callable, we assume it is a
#     # string representing the edge attribute containing the weight of
#     # the edge.
#     if G.is_multigraph():
#         print 'multigraph'
#         return lambda u, v, d: min(attr.get(weight, 1) for attr in d.values())
#     print 'neither'
#     return lambda u, v, data: data.get(weight, 1)