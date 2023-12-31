import networkx as nx
import matplotlib.pyplot as plt

def draw_graph(graph):

    # extract nodes from graph
    nodes = set([n1 for n1, n2 in graph] + [n2 for n1, n2 in graph])

    # create networkx graph
    G=nx.Graph()

    # add nodes
    for node in nodes:
        G.add_node(node)

    # add edges
    for edge in graph:
        G.add_edge(edge[0], edge[1])

    # draw graph
    pos = nx.shell_layout(G)
    nx.draw(G, pos)

    # show graph
    plt.show()

# draw example
graph = [(20, 21),(21, 22),(22, 23), (23, 24),(24, 25), (25, 20)]
# draw_graph(graph)



test=nx.Graph()

test.add_node(0)
test.add_node(1)
test.add_node(2)

test.add_edge(0, 1, weight=1)
test.add_edge(1, 0, weight=1)
test.add_edge(0, 2, weight=2)
test.add_edge(2, 0, weight=2)
test.add_edge(1, 2, weight=2)
test.add_edge(2, 1, weight=2)

labels = range(3)

pos = nx.shell_layout(test)
nx.draw(test, pos, with_labels=True)

plt.show()
