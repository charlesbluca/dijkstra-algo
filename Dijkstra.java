/**
* Dijkstra's Algorithm
* works in conjunction with Display.java to visualize shortest path
* @author Charles Blackmon-Luca, ccb2158
* @author some data structures TA probably
*/

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import java.io.FileReader;
import java.io.BufferedReader;

public class Dijkstra {

  // Keep a fast index to nodes in the map
  private Map<String, Vertex> vertexNames;
  
  /**
   * Construct an empty Dijkstra with a map. The map's key is the name of a vertex
   * and the map's value is the vertex object.
   */
  public Dijkstra() {
    vertexNames = new HashMap<String, Vertex>();
  }

  /**
   * Adds a vertex to the dijkstra. Throws IllegalArgumentException if two vertices
   * with the same name are added.
   * 
   * @param v
   *          (Vertex) vertex to be added to the dijkstra
   */
  public void addVertex(Vertex v) {
    if (vertexNames.containsKey(v.name))
      throw new IllegalArgumentException("Cannot create new vertex with existing name.");
    vertexNames.put(v.name, v);
  }

  /**
   * Gets a collection of all the vertices in the dijkstra
   * 
   * @return (Collection<Vertex>) collection of all the vertices in the dijkstra
   */
  public Collection<Vertex> getVertices() {
    return vertexNames.values();
  }

  /**
   * Gets the vertex object with the given name
   * 
   * @param name
   *          (String) name of the vertex object requested
   * @return (Vertex) vertex object associated with the name
   */
  public Vertex getVertex(String name) {
    return vertexNames.get(name);
  }

  /**
   * Adds a directed edge from vertex u to vertex v
   * 
   * @param nameU
   *          (String) name of vertex u
   * @param nameV
   *          (String) name of vertex v
   * @param cost
   *          (double) cost of the edge between vertex u and v
   */
  public void addEdge(String nameU, String nameV, Double cost) {
    if (!vertexNames.containsKey(nameU))
      throw new IllegalArgumentException(nameU + " does not exist. Cannot create edge.");
    if (!vertexNames.containsKey(nameV))
      throw new IllegalArgumentException(nameV + " does not exist. Cannot create edge.");
    Vertex sourceVertex = vertexNames.get(nameU);
    Vertex targetVertex = vertexNames.get(nameV);
    Edge newEdge = new Edge(sourceVertex, targetVertex, cost);
    sourceVertex.addEdge(newEdge);
  }
  
  /**
   * Adds an undirected edge between vertex u and vertex v by adding a directed
   * edge from u to v, then a directed edge from v to u
   * 
   * @param nameU
   *          (String) name of vertex u
   * @param nameV
   *          (String) name of vertex v
   * @param cost
   *          (double) cost of the edge between vertex u and v
   */
  public void addUndirectedEdge(String nameU, String nameV, double cost) {
    addEdge(nameU, nameV, cost);
    addEdge(nameV, nameU, cost);
  }

  // STUDENT CODE STARTS HERE

  /**
   * Computes the euclidean distance between two points as described by their
   * coordinates
   * 
   * @param ux
   *          (double) x coordinate of point u
   * @param uy
   *          (double) y coordinate of point u
   * @param vx
   *          (double) x coordinate of point v
   * @param vy
   *          (double) y coordinate of point v
   * @return (double) distance between the two points
   */
  public double computeEuclideanDistance(double ux, double uy, double vx, double vy) {
      // dist = sqrt( ( ux - vx )^2 + ( uy - vy )^2 )
      return Math.sqrt(Math.pow((ux - vx),2) + Math.pow((uy - vy),2));
  }
  
  /**
   * Calculates the euclidean distance for all edges in the map using the
   * computeEuclideanCost method.
   */
  public void computeAllEuclideanDistances() {
      // iterate through all vertices
      Collection<Vertex> c = getVertices();
      for(Vertex w : c) {
          // iterate through all edges
          for(Edge e : w.adjacentEdges) {
              Vertex u = e.source;
              Vertex v = e.target;
              // get coordinates of source and target
              double ux = u.x;
              double uy = u.y;
              double vx = v.x;
              double vy = v.y;
              // calculate distance and assign it to edge
              double dist = computeEuclideanDistance(ux, uy, vx, vy);
              e.distance = dist;
          }
      }
  }

  /**
   * Dijkstra's Algorithm. 
   * 
   * @param s
   *          (String) starting city name
   */
  public void doDijkstra(String s) {
      // start all vertices with infinite distance + unknown
      Collection<Vertex> c = getVertices();
      for(Vertex v : c) {
          v.distance = Integer.MAX_VALUE;
          v.known = false;
      }
      // set source vertex distance to 0
      Vertex source = getVertex(s);
      source.distance = 0;
      // loop until all paths are known
      while(isUnknownIn(c)) {
          // find closest unknown vertex by iterating through vertices
          Vertex min = null;
          double minDist = Integer.MAX_VALUE;
          for(Vertex v : c) {
              if(!v.known && v.distance < minDist) {
                  min = v;
                  minDist = v.distance;
              }
          }
          // set vertex path to known
          min.known = true;
          // iterate through its neighbors
          for(Edge e : min.adjacentEdges) {
              Vertex n = e.target;
              // check if neighbor is unknown
              if(!n.known) {
                  double cost = e.distance;
                  // check if we have found a shorter path
                  if(min.distance + cost < n.distance) {
                      // update n distance
                      n.distance = min.distance + cost;
                      n.prev = min;
                  }
              }
          }
      }
  }

  /**
   * Checks for unknown shortest path in a graph.
   * @return true if some vertex is unknown, false if not
   */
  public boolean isUnknownIn(Collection<Vertex> c) {
      // iterate through vertices
      for(Vertex v : c) {
          // if there is an unknown return true
          if(v.known == false)
              return true;
	  }
      // else return false
      return false;
  }

  /**
   * Returns a list of edges for a path from city s to city t. This will be the
   * shortest path from s to t as prescribed by Dijkstra's algorithm
   * 
   * @param s
   *          (String) starting city name
   * @param t
   *          (String) ending city name
   * @return (List<Edge>) list of edges from s to t
   */
  public List<Edge> getDijkstraPath(String s, String t) {
      // get all dijkstra paths
      doDijkstra(s);
      // create list for edges
      List<Edge> path = new LinkedList<>();
      // initialize vertices for current and source (s)
      Vertex current = getVertex(t);
      Vertex source = getVertex(s);
      Vertex prev = null;
      // iterate until we reach the source vertex
      while(!current.equals(source)) {
          // mark down previous vertex in path
          prev = current.prev;
          // look for edge pointing to this vertex
          for(Edge e : current.adjacentEdges) {
              if(e.target == prev) {
                  path.add(0,e); // add edge to list
              }
              // set prev to new current
              current = prev;
          }
      }
      // print out path and total distance
      double distance = 0;
      System.out.print(source.name);
      for(Edge e : path) {
          System.out.print(" -> " + e.source.name);
          distance += e.distance;
      }
      System.out.printf("\ndistance: %1$.1f \n", distance);
      return path;
  }

  // STUDENT CODE ENDS HERE

  /**
   * Prints out the adjacency list of the dijkstra for debugging
   */
  public void printAdjacencyList() {
    for (String u : vertexNames.keySet()) {
      StringBuilder sb = new StringBuilder();
      sb.append(u);
      sb.append(" -> [ ");
      for (Edge e : vertexNames.get(u).adjacentEdges) {
        sb.append(e.target.name);
        sb.append("(");
        sb.append(e.distance);
        sb.append(") ");
      }
      sb.append("]");
      System.out.println(sb.toString());
    }
  }


  /** 
   * A main method that illustrates how the GUI uses Dijkstra.java to 
   * read a map and represent it as a graph. 
   * You can modify this method to test your code on the command line. 
   */
  public static void main(String[] argv) throws IOException {
    String vertexFile = "cityxy.txt"; 
    String edgeFile = "citypairs.txt";

    Dijkstra dijkstra = new Dijkstra();
    String line;

    // Read in the vertices
    BufferedReader vertexFileBr = new BufferedReader(new FileReader(vertexFile));
    while ((line = vertexFileBr.readLine()) != null) {
      String[] parts = line.split(",");
      if (parts.length != 3) {
        vertexFileBr.close();
        throw new IOException("Invalid line in vertex file " + line);
      }
      String cityname = parts[0];
      int x = Integer.valueOf(parts[1]);
      int y = Integer.valueOf(parts[2]);
      Vertex vertex = new Vertex(cityname, x, y);
      dijkstra.addVertex(vertex);
    }
    vertexFileBr.close();

    BufferedReader edgeFileBr = new BufferedReader(new FileReader(edgeFile));
    while ((line = edgeFileBr.readLine()) != null) {
      String[] parts = line.split(",");
      if (parts.length != 3) {
        edgeFileBr.close();
        throw new IOException("Invalid line in edge file " + line);
      }
      dijkstra.addUndirectedEdge(parts[0], parts[1], Double.parseDouble(parts[2]));
    }
    edgeFileBr.close();

    // Compute distances. 
    // This is what happens when you click on the "Compute All Euclidean Distances" button.
    dijkstra.computeAllEuclideanDistances();
    
    // print out an adjacency list representation of the graph
    dijkstra.printAdjacencyList();

    // This is what happens when you click on the "Draw Dijkstra's Path" button.

    // In the GUI, these are set through the drop-down menus.
    String startCity = "SanFrancisco";
    String endCity = "Boston";

    // Get weighted shortest path between start and end city. 
    List<Edge> path = dijkstra.getDijkstraPath(startCity, endCity);
    
    System.out.print("Shortest path between "+startCity+" and "+endCity+": ");
    System.out.println(path);
  }

}
