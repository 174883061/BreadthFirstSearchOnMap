/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private static HashMap<GeographicPoint, MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		vertices = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		Set<GeographicPoint> pointSet = vertices.keySet();
		int numEdges = 0;
		for(GeographicPoint point : pointSet){
			numEdges += vertices.get(point).getNumEdges();
		}
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(location == null || vertices.containsKey(location)){
			return false;
		}
		MapNode node = new MapNode(location);
		vertices.put(location, node);
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
//		System.out.println("~~~~~~~~~~~~~addEdge");
		//TODO: Implement this method in WEEK 3
		if(from == null || to == null || roadName == null || roadType == null || length < 0 ||
				!vertices.containsKey(from) || !vertices.containsKey(to)){
			throw new IllegalArgumentException();
		}
		//这是一个directed graph，每次添加edge的时候需要添加双向2条，因为要求说每对nodes中有2个edge?
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		MapNode node = vertices.get(from);
		node.getEdges().add(edge); //在原先的edge list里面添加一个edge
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		if(start == null || goal == null){
			System.out.println("Start or goal node is null! No path exists.");
			return new LinkedList<GeographicPoint>();
		}
//		System.out.println("start: " + vertices.containsKey(start));
//		System.out.println("goal: " + vertices.containsKey(goal));
		if(!vertices.containsKey(start) || !vertices.containsKey(goal)){
			System.out.println("Start or goal node not in the graph! No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		HashMap<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		boolean found = bfsSearch(start, goal, parentMap, nodeSearched);
		if (!found) {
			System.out.println("No path exists");
			return null;
		}
		return constructPath(start, goal, parentMap);
	}

	private boolean bfsSearch(GeographicPoint start, GeographicPoint goal,
							  HashMap<GeographicPoint, GeographicPoint> parentMap, Consumer<GeographicPoint> nodeSearched){
		HashSet<GeographicPoint> visitedSet = new HashSet<>();
		Queue<GeographicPoint> toExploreQueue = new LinkedList<>();
		toExploreQueue.add(start);
		boolean found = false;
		while (!toExploreQueue.isEmpty()) {
			GeographicPoint currPoint = toExploreQueue.remove();
			// Hook for visualization.  See writeup.
			nodeSearched.accept(currPoint);
			if (currPoint.equals(goal)) {
				found = true;
				break;
			}
//			MapNode node = vertices.get(currPoint);
//			List<MapEdge> edges = node.getEdges();
//			List<GeographicPoint> ns = node.getNeighbors(currPoint);

			List<GeographicPoint> neighbors = vertices.get(currPoint).getNeighbors(currPoint);
			//System.out.println("neighbors.size() = " + neighbors.size());
			ListIterator<GeographicPoint> it = neighbors.listIterator();
			while (it.hasNext()) {
				GeographicPoint next = it.next();
				if (!visitedSet.contains(next)) {
					visitedSet.add(next);
					parentMap.put(next, currPoint);
					toExploreQueue.add(next);
				}
			}
		}
		return found;
	}

	private List<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
												HashMap<GeographicPoint, GeographicPoint> parentMap){
		LinkedList<GeographicPoint> path = new LinkedList<>();
		GeographicPoint curr = goal;
		while (!curr.equals(start)) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(start);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
//		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		GraphLoader.loadRoadMap("data/graders/mod2/map3.txt", firstMap);
//		System.out.println("DONE.");
		System.out.println("firstMap.getNumVertices() = " + firstMap.getNumVertices());
		System.out.println("firstMap.getNumEdges() = " + firstMap.getNumEdges());
//		System.out.println("firstMap.getVertices() = " + firstMap.getVertices());
		GeographicPoint start = new GeographicPoint(1, 2);
		GeographicPoint goal = new GeographicPoint(0, 0);
		System.out.println("::::::::::::::::::::::::::::::::::::::::::");
		System.out.println("=======" + firstMap.bfs(start, goal));

//		for(GeographicPoint point : vertices){
//			System.out.println(vertices);
//		}
		// You can use this method for testing.
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/
		
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
