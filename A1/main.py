import xml.etree.ElementTree as ET
import random
import time
import networkx as nx
import matplotlib.pyplot as plt

from typing import List

longestNum = 0

# Adjustable parameters
MUTATIONS = 2
POP_SIZE = 100
GENERATIONS = 50
SURVIVORS = 1


class Edge:
    def __init__(self, dest, value) -> None:
        self.dest = dest
        self.value = value

    def __str__(self):
        return str(self.value)


class Vertex:
    def __init__(self, id, edges: List[Edge]) -> None:
        self.id = id
        self.edges = edges

    def __setitem__(self, key, value):
        self.edges

    def __getitem__(self, item):
        return self.edges

    def __str__(self):
        global longestNum
        res = ""
        iteration = 0
        for i in range(len(self.edges) + 1):
            if self.id == i:
                x = "X"
                res += f"{x:<{longestNum}}"
            else:
                length = len(f"{str(self.edges[iteration])}")
                res += f"{str(self.edges[iteration]):<{longestNum}}"
                iteration += 1

        return res + "\n"


class Graph:
    def __init__(self, vertices: List[Vertex], name, source, description) -> None:
        self.vertices = vertices
        self.name = name
        self.source = source
        self.description = description

    def __str__(self):
        print("########################################################")
        res = f"Name: {self.name}\nSource: {self.source}\nDescription: {self.description}\nVertex\t"
        strVertex = ""
        for i, value in enumerate(self.vertices):
            res += f"{i:<{longestNum}}"
            if i < 10:
                strVertex += "  "
            elif i < 100:
                strVertex += " "
            strVertex += f"V{i}:\t{str(value)}"

        return res + "\n" + strVertex


def initXML(filename) -> Graph:
    # Print information and start taking time
    global longestNum
    print("Starting parsing xml file")
    t0 = time.time()

    # Reading the file and setting the root
    tree = ET.parse("src/" + filename)
    root = tree.getroot()

    # Extracting graph from xml file
    graphXML = root[5]
    graphVertices = []

    # Loop all Vertices
    for i, vert in enumerate(graphXML):
        # Loop and get all edges for vertex
        vertEdges = []
        for edge in vert:
            # TODO bs check depends on construction of xml data and should be changed but not sure how yet
            length = len(str(float(edge.attrib.get('cost')))) + 1
            if float(edge.attrib.get('cost')) < 9990.0:
                vertEdges.append(Edge(int(edge.text), float(edge.attrib.get('cost'))))
            if length > longestNum:
                longestNum = length

        # Create Vertex and add Edges list
        graphVertices.append(Vertex(i, vertEdges))

    # Create Graph object and add Vertices
    resGraph = Graph(graphVertices, root[0].text, root[1].text, root[2].text)

    # Print information and construction time
    t1 = time.time()
    total = t1 - t0
    print(f"Graph {root[0].text} constructed in {total}ms")

    return resGraph


class Client:
    def __init__(self, graph: Graph, stops=[], weighting = -1.0, selecter = -1, revers = False) -> None:
        self.totalWeight = None
        self.stops = []
        self.weights = []
        self.weighting = weighting
        self.selecter = selecter
        self.reverse = revers
        if len(stops) == 0:
            if weighting != -1:
                self.initialize_weighted(graph)
            elif selecter != -1:
                self.initialize_selecter(graph)
            else:
                self.initialize_greedy(graph)
        else:
            self.calculate_weight(graph)
        self.completed_weight()

    def __str__(self) -> str:
        res = "########################################################\nStops in Graph\n"
        for i in self.stops:
            res += f"{i}\t"
        res += "\nWeight in between\n"

        self.totalWeight = 0
        for i in self.weights:
            res += f"{i}\t"
            self.totalWeight += i
        res += f"\nTotal weight\n{self.totalWeight}\n"

        return res

    def show_graph(self, graph: Graph, generation):
        G=nx.Graph()

        for i, vertex in enumerate(graph.vertices):
            G.add_node(i)

        
        for i, vertex in enumerate(graph.vertices):
            for edge in vertex.edges:
                G.add_edge(i, edge.dest,color='#78D1FA', weight=edge.value/(max(edge.value for edge in vertex.edges)/3)+1)

        # G[0][1]['color'] = 'r'
        for i in range(len(self.stops) - 1):
            G[self.stops[i]][self.stops[i + 1]]['color'] = 'r'

        edges = G.edges()
        colors = [G[u][v]['color'] for u,v in edges]
        weights = [G[u][v]['weight'] for u,v in edges]

        pos = nx.shell_layout(G)
        plt.figure(generation)
        fig_manager = plt.get_current_fig_manager()
        fig_manager.set_window_title(f"Generation {generation}")
        nx.draw(G, pos, edge_color=colors, width=weights, with_labels=True)

    def initialize_greedy(self, graph: Graph):
        # Get random start point
        next_vertex = random.randint(0, len(graph.vertices) - 1)

        # Add first vertex
        self.stops.append(next_vertex)
        # Loop to get all vertices
        for _ in range(len(graph.vertices) - 1):
            # Find a 0 weight edge or the best
            lowest_edge_weight = 10000000000000000
            best_dest = -1
            for i in graph.vertices[next_vertex].edges:
                if i.dest not in self.stops:
                    if i.value <= 0.0:
                        next_vertex = i.dest
                        break
                    else:
                        if lowest_edge_weight > i.value:
                            lowest_edge_weight = i.value
                            next_vertex = i.dest
            # Add the best found edge
            self.stops.append(next_vertex)

        # Add the takeback move
        self.stops.append(self.stops[0])
        self.calculate_weight(graph)

    # Idee 1 umsetzung
    def initialize_weighted(self, graph: Graph):
        def get_weight(edge: Edge):
            return edge.value

        def get_available_edges(edges: list[Edge]):
            res = []
            for i in edges:
                if i.dest not in self.stops:
                    res.append(i)
            return res

        # Get random start point
        next_vertex = random.randint(0, len(graph.vertices) - 1)

        # Add first vertex
        self.stops.append(next_vertex)
        # Loop to get all vertices
        for _ in range(len(graph.vertices) - 1):
            available_edges = get_available_edges(graph.vertices[next_vertex].edges)
            available_edges.sort(key=get_weight)

            compare_weight = available_edges[-1].value * self.weighting

            lowest_distance_edge: Edge = None
            for i in available_edges:
                if i.value == compare_weight:
                    next_vertex = i.dest
                    lowest_distance_edge = None
                    break
                else:
                    if lowest_distance_edge is None or abs(compare_weight-i.value) < abs(compare_weight-lowest_distance_edge.value):
                        lowest_distance_edge = i

            if lowest_distance_edge is not None:
                next_vertex = lowest_distance_edge.dest

            self.stops.append(next_vertex)

        # Add the takeback move
        self.stops.append(self.stops[0])
        self.calculate_weight(graph)

    # Idee 1 umsetzung
    def initialize_selecter(self, graph: Graph):
        def get_weight(edge: Edge):
            return edge.value

        def get_available_edges(edges):
            res = []
            for i in edges:
                if i.dest not in self.stops:
                    res.append(i)
            return res

        # Get random start point
        next_vertex = random.randint(0, len(graph.vertices) - 1)

        # Add first vertex
        self.stops.append(next_vertex)
        # Loop to get all vertices
        for _ in range(len(graph.vertices) - 1):
            available_edges: list[Edge] = get_available_edges(graph.vertices[next_vertex].edges)
            available_edges.sort(key=get_weight, reverse=self.reverse)

            if self.selecter > len(available_edges):
                next_vertex = available_edges[-1].dest
            else:
                next_vertex = available_edges[self.selecter - 1].dest

            self.stops.append(next_vertex)

        # Add the takeback move
        self.stops.append(self.stops[0])
        self.calculate_weight(graph)

    def calculate_weight(self, graph: Graph):
        self.weights.clear()
        # Loop all stops to get new weight
        for i in range(len(self.stops)):
            # Prevent list out of bound while looping stops list and trying to access next stop
            if not (len(self.weights) > 1 and self.stops[i] is self.stops[0]):
                # Loop all vertex edges and find the correct next one and add the wight in between
                source = graph.vertices[self.stops[i]]
                for e in source.edges:
                    if self.stops[i + 1] is e.dest:
                        self.weights.append(e.value)
                        break

    def completed(self) -> bool:
        if len(self.stops) != 0:
            return True
        else:
            return False

    def completed_weight(self):
        self.totalWeight = sum(self.weights)

    def variation(self):
        if not self.completed():
            return
        else:
            # TODO Variation Logic
            pass


def sort_filter(c: Client):
    return c.totalWeight


class Population:
    def __init__(self, graph: Graph, show_graphs: bool):
        t0 = time.time()
        self.top_dog: Client
        self.graph = graph
        self.current_generation = 1
        self.show_graphs = show_graphs
        self.clients = []
        for x in range(POP_SIZE):
            temp = Client(graph, weighting=random.random())
            self.clients.append(temp)

        self.fittest()
        self.bestVal = self.top_dog.totalWeight
        t1 = time.time()
        self.gen_time = t1 - t0
        self.totalTime = 0
        print(self)

        while self.current_generation < GENERATIONS:
            self.new_gen()

    def __str__(self):
        res = f"Generation: {self.current_generation}\n\n"
        res += f"Time required for generation: \033[91m{self.gen_time} ms\033[00m\n\n"
        res += f"Survivors of this generation\n_________________________________________________________\n\n"
        res += f"Fittest individual:\n\033[92m{self.top_dog}\033[00m\n\n"
        if self.show_graphs:
            self.top_dog.show_graph(self.graph, self.current_generation)
        if SURVIVORS > 1:
            res += "Others:\n"
            for i in range(SURVIVORS - 1):
                res += f"\033[93m{self.clients[i]}\033[00m\n"
        res += f"Best Value of this population: \033[94m{self.bestVal}\033[00m\n\n"
        return res


    def fittest(self):
        self.top_dog = self.clients[0]
        for x in self.clients:
            if x.totalWeight < self.top_dog.totalWeight:
                self.top_dog = x
        return

    # Selection based on highest fitness (lowest weight)
    def selection(self):
        surviors = []
        self.clients.sort(key=sort_filter)
        for x in range(SURVIVORS):
            surviors.append(self.clients[x])

        return surviors

    def mutation(self):
        # Mutate by switching 2 verts in the list
        for c in self.clients:
            counter = 0
            while counter < MUTATIONS:
                length = len(c.stops)
                rand1 = random.randint(1, length - 2)
                rand2 = 0
                while rand2 == 0 or rand2 == rand1:
                    rand2 = random.randint(1, length - 2)

                temp = c.stops[rand1]
                c.stops[rand1] = c.stops[rand2]
                c.stops[rand2] = temp
                c.calculate_weight(self.graph)
                c.completed_weight()
                counter += 1

    # Initialize new generation by killing all unfit Clients,
    # repopulating with start points randomly selected from the path of the survivors
    def new_gen(self):
        t0 = time.time()
        self.current_generation += 1
        self.clients = self.selection()
        self.mutation()

        while POP_SIZE > len(self.clients):
            new_weighing: float
            if len(self.clients) > 0:
                mom = self.clients[random.randint(0, len(self.clients)-1)]
                if len(self.clients) == 1:
                    dad = self.clients[random.randint(0, len(self.clients)-1)]

                    new_weighing = mom.weighting + dad.weighting
                    if mom.totalWeight < dad.totalWeight:
                        new_weighing += mom.weighting
                    else:
                        new_weighing += dad.weighting
                else:
                    new_weighing = mom.weighting * 2 + random.random()

                new_weighing /= 3
            else:
                new_weighing = random.random()
            temp = Client(self.graph, weighting=new_weighing)
            self.clients.append(temp)

        self.fittest()
        if self.top_dog.totalWeight < self.bestVal:
            self.bestVal = self.top_dog.totalWeight
        t1 = time.time()
        print(self)

# Main Function (Entry point)
if __name__ == "__main__":
    br17 = initXML("br17.xml")
    print(br17)
    Population(br17, True)
    plt.show()
    # test_client = Client(br17, weighting=0.07)
    # print(test_client)
    # test_client.show_graph(br17)
