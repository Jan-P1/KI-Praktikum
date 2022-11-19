import xml.etree.ElementTree as ET
import random
import time

from typing import List

longestNum = 0

# Adjustable parameters
MUTATIONS = 1
POP_SIZE = 20


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
    def __init__(self, graph: Graph, stops=[]) -> None:
        # Ich glaube, es wäre um einiges effizienter, wenn der graph nicht in den Clients gespeichert wird
        self.graph = graph
        self.stops = []
        self.weights = []
        if len(stops) == 0:
            self.initialize_random()
        else:
            self.calculate_weight()

    def __str__(self) -> str:
        totalWeight = 0
        res = "########################################################\nStops in Graph\n"
        for i in self.stops:
            res += f"{i}\t"
        res += "\nWeight in between\n"

        for i in self.weights:
            res += f"{i}\t"
            totalWeight += i
        res += f"\nTotal weight\n{totalWeight}\n"

        return res

    def initialize_random(self):
        for _ in self.graph.vertices:
            is_duplicate = True
            while (is_duplicate):
                new_random_vertex = random.randint(0, len(self.graph.vertices) - 1)
                if new_random_vertex not in self.stops:
                    self.stops.append(new_random_vertex)
                    is_duplicate = False
        self.stops.append(self.stops[0])
        self.calculate_weight()

    def calculate_weight(self):
        self.weights.clear()
        # Loop all stops to get new weight
        for i in range(len(self.stops)):
            # Prevent list out of bound while looping stops list and trying to access next stop
            if not (len(self.weights) > 1 and self.stops[i] is self.stops[0]):
                # Loop all vertex edges and find the correct next one and add the wight in between
                source = self.graph.vertices[self.stops[i]]
                for e in source.edges:
                    if self.stops[i + 1] is e.dest:
                        self.weights.append(e.value)
                        break

    def completed(self) -> bool:
        if len(self.graph) == len(self.stops):
            return True
        else:
            return False

    def completed_weight(self) -> int:
        return sum(self.weights)

    def mutation(self):
        if not self.completed():
            return
        else:
            # Mutate by switching 2 verts in the list
            counter = 0
            while counter < MUTATIONS:
                counter += 1
                length = len(self.stops)
                rand1 = random.randint(1, length - 2)
                rand2 = 0
                while rand2 == 0 or rand2 == rand1:
                    rand2 = random.randint(1, length - 2)

                temp = self.stops[rand1]
                self.stops[rand1] = self.stops[rand2]
                self.stops[rand2] = temp
                self.calculate_weight()
            return


    def variation(self):
        if not self.completed():
            return
        else:
            # TODO Variation Logic
            pass


class Population:
    def __init__(self):
        self.topDog = None
        self.clients = []
        for x in range(POP_SIZE):
            temp = Client()
            temp.initialize_random()
            self.clients.append(temp)

        self.fittest()

    def fittest(self):
        self.topDog = self.clients[0]
        for x in self.clients:
            if x.calculate_weight() < self.topDog.calculate_weight():
                self.topDog = x
        return


# Main Function (Entry point)
if __name__ == "__main__":
    a280 = initXML("a280.xml")
    print(a280)

    br17 = initXML("br17.xml")
    print(br17)
    print(longestNum)

    test_client = Client(br17)
    print(test_client)

    # Wer lust hat, kann das ja mal ausprobieren xD
    # a280 = initXML("a280.xml")
    # print(a280)
    # test obs klapt <- pls ignore