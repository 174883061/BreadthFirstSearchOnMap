package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;

public class MapNode {
    private GeographicPoint location;
    private List<MapEdge> edges;

    public int getNumEdges(){
        return edges.size();
    }

    public MapNode(GeographicPoint location, List<MapEdge> edges) {
        this.location = location;
        this.edges = edges;
    }
    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new ArrayList<>();
    }

    //返回与指定point相邻的所有点
    public List<GeographicPoint> getNeighbors(GeographicPoint location) {
        if(location == null){
            return new ArrayList<>();
        }
        List<GeographicPoint> neighbors = new ArrayList<>();
//        System.out.println("edges.size() = " + edges.size());
        for(MapEdge edge : edges){
            if(location.equals(edge.getStart())){
                neighbors.add(edge.getEnd());
            }
            if(location.equals(edge.getEnd())){
                neighbors.add(edge.getStart());
            }
        }
        return neighbors;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public void setLocation(GeographicPoint location) {
        this.location = location;
    }

    public List<MapEdge> getEdges() {
        return edges;
    }

    public void setEdges(List<MapEdge> edges) {
        this.edges = edges;
    }
}
