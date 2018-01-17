# dijkstra-algo
An assignment for Data Structures in Java; uses Dijkstra's algorithm to find the shortest distance between two cities, with a GUI representation (credited to my professor and IAs at the time)

Compiling/Executing:
----------
- In command line, type "javac Display.java" to compile all files
- Then type "java Display" to run the GUI - from there the interface should be an adequate guide

Design Choices:
----------
- For computeEuclideanDistance, the standard Euclidean distance formula was used
- For exponents and square roots, I used methods in the Math class
- For computeAllEuclideanDistances, I iterated through the collection of vertices and all their edges
- Arguments were then taken from the source and target vertices
- doDijkstra follows the standard algorithm from the textbook
- A helper method was added to check if there was still a vertex with an unknown shortest path
- An iterative algorithm was used to search for the next closest vertex in the collection, by looking through the whole collection for unknown vertices with the smallest distance
- For getDijkstraPath, the doDijkstra method is used to get the shortest paths for all vertices to one source "s"
- From there, we iterate from the target vertex back to the source, adding all the vertices on their Dijkstra path to a LinkedList
- Each edge is placed at the beginning of the list to retain the order of the path
- This path is then printed with the associated distance

Quirks:
----------
- The total distance printed may vary slightly (around .1) from the distance that can be acquired by adding up the displayed edge weights
- This is because my Euclidean computations do NOT round to the first deci
