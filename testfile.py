

class Graph:
    def __init__(self, vertices, name, source, description) -> None:
        self.vertices = vertices
        self.name = name
        self.source = source
        self.description = description
    
    def __str__(self):
        res = f"Name: {self.name}\nSource: {self.source}\nDescription: {self.description}\nVertex\t"
        strVertex = ""
        for i, value in enumerate(self.vertices):
            res += f"{i}\t"
            strVertex += f"V{i}:\t{str(value)}"

        return res + "\n" + strVertex

class Vertex:
    def __init__(self, id, edges) -> None:
        self.id = id
        self.edges = edges

    def __str__(self):
        res = ""
        itter = 0
        for i in range(len(self.edges) + 1):
            if self.id == i:
                res += "X \t"
            else:
                res += f"{str(self.edges[itter])}\t"
                itter += 1

        return res + "\n"

class Edge:
    def __init__(self, dest, value) -> None:
        self.value = value
        self.dest = dest

    def __str__(self):
        return str(self.value)

newEdge1 = Edge(1, 1)
newEdge2 = Edge(0, 1)
newVertex1 = Vertex(0, [newEdge1])
newVertex2 = Vertex(1, [newEdge2])

newGraph = Graph([newVertex1, newVertex2], "Jan", "Okan", "Beide behindert")
print(newGraph)