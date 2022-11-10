import xml.etree.ElementTree as ET
import time
import math
longestNum = 0


class Graph:
    def __init__(self, vertices, name, source, description) -> None:
        self.vertices = vertices
        self.name = name
        self.source = source
        self.description = description

    def __str__(self):
        print("########################################################")
        res = f"Name: {self.name}\nSource: {self.source}\nDescription: {self.description}\nVertex\t"
        strVertex = ""
        for i, value in enumerate(self.vertices):
            res += f"{i}"
            res += "\t" * (math.floor(longestNum / 4) + 1)
            if i < 10:
                strVertex += "  "
            elif i < 100:
                strVertex += " "
            strVertex += f"V{i}:\t{str(value)}"

        return res + "\n" + strVertex


class Vertex:
    def __init__(self, id, edges) -> None:
        self.id = id
        self.edges = edges

    def __str__(self):
        global longestNum
        res = ""
        iteration = 0
        for i in range(len(self.edges) + 1):
            if self.id == i:
                res += "X\t\t"
            else:
                length = len(f"{str(self.edges[iteration])}")
                res += f"{str(self.edges[iteration])}"
                res += "\t" * (math.floor(longestNum / 4) - math.floor(length / 4) + 1)
                iteration += 1

        return res + "\n"


class Edge:
    def __init__(self, dest, value) -> None:
        self.dest = dest
        self.value = value

    def __str__(self):
        return str(self.value)


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
            length = len(str(float(edge.attrib.get('cost'))))
            if float(edge.attrib.get('cost')) < 9990.0:
                vertEdges.append(Edge(edge.text, float(edge.attrib.get('cost'))))
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


# Main Function (Entry point)
if __name__ == "__main__":
    br17 = initXML("br17.xml")
    print(br17)

    # Wer lust hat, kann das ja mal ausprobieren xD
    # a280 = initXML("a280.xml")
    # print(a280)
