import java.util.*;

// Vertex class represents a graph node with adjacency information
class Vertex<V> {
    private V data;
    private Map<Vertex<V>, Double> adjacentVertices = new HashMap<>();

    public Vertex(V data) {
        this.data = data;
    }

    public V getData() {
        return data;
    }

    public Map<Vertex<V>, Double> getAdjacentVertices() {
        return adjacentVertices;
    }

    public void addAdjacentVertex(Vertex<V> destination, double weight) {
        adjacentVertices.put(destination, weight);
    }

    @Override
    public String toString() {
        return data.toString();
    }
}

// WeightedGraph class manages a list of vertices
class WeightedGraph<V> {
    private List<Vertex<V>> vertices = new ArrayList<>();

    public void addVertex(Vertex<V> vertex) {
        vertices.add(vertex);
    }

    public List<Vertex<V>> getVertices() {
        return vertices;
    }
}

// Search interface defines a method to find a path between two vertices
interface Search<V> {
    List<Vertex<V>> getPath(Vertex<V> start, Vertex<V> end);
}

// BreadthFirstSearch class implements BFS algorithm
class BreadthFirstSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;

    public BreadthFirstSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }

    public List<Vertex<V>> getPath(Vertex<V> start, Vertex<V> end) {
        Map<Vertex<V>, Vertex<V>> previous = new HashMap<>();
        Queue<Vertex<V>> queue = new LinkedList<>();
        Set<Vertex<V>> visited = new HashSet<>();

        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            Vertex<V> current = queue.poll();
            if (current.equals(end)) break;

            for (Vertex<V> neighbor : current.getAdjacentVertices().keySet()) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    previous.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        List<Vertex<V>> path = new LinkedList<>();
        for (Vertex<V> at = end; at != null; at = previous.get(at)) {
            path.add(0, at);
        }
        return path;
    }
}

// DijkstraSearch class implements Dijkstra's shortest path algorithm
class DijkstraSearch<V> implements Search<V> {
    private WeightedGraph<V> graph;

    public DijkstraSearch(WeightedGraph<V> graph) {
        this.graph = graph;
    }

    public List<Vertex<V>> getPath(Vertex<V> start, Vertex<V> end) {
        Map<Vertex<V>, Double> distances = new HashMap<>();
        Map<Vertex<V>, Vertex<V>> previous = new HashMap<>();
        PriorityQueue<Vertex<V>> queue = new PriorityQueue<>(Comparator.comparing(distances::get));

        for (Vertex<V> vertex : graph.getVertices()) {
            distances.put(vertex, Double.POSITIVE_INFINITY);
        }
        distances.put(start, 0.0);
        queue.add(start);

        while (!queue.isEmpty()) {
            Vertex<V> current = queue.poll();

            if (current.equals(end)) break;

            for (Map.Entry<Vertex<V>, Double> neighborEntry : current.getAdjacentVertices().entrySet()) {
                Vertex<V> neighbor = neighborEntry.getKey();
                double newDist = distances.get(current) + neighborEntry.getValue();
                if (newDist < distances.get(neighbor)) {
                    distances.put(neighbor, newDist);
                    previous.put(neighbor, current);
                    queue.add(neighbor);
                }
            }
        }

        List<Vertex<V>> path = new LinkedList<>();
        for (Vertex<V> at = end; at != null; at = previous.get(at)) {
            path.add(0, at);
        }
        return path;
    }
}

// Main class to demonstrate usage
public class Main {
    public static void main(String[] args) {
        Vertex<String> a = new Vertex<>("A");
        Vertex<String> b = new Vertex<>("B");
        Vertex<String> c = new Vertex<>("C");

        a.addAdjacentVertex(b, 1);
        b.addAdjacentVertex(c, 2);
        a.addAdjacentVertex(c, 4);

        WeightedGraph<String> graph = new WeightedGraph<>();
        graph.addVertex(a);
        graph.addVertex(b);
        graph.addVertex(c);

        Search<String> bfs = new BreadthFirstSearch<>(graph);
        Search<String> dijkstra = new DijkstraSearch<>(graph);

        System.out.println("BFS Path: " + bfs.getPath(a, c));
        System.out.println("Dijkstra Path: " + dijkstra.getPath(a, c));
    }
}
