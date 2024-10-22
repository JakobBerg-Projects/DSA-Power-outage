package student;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import graph.*;
import java.util.Stack; 


public class ProblemSolver implements IProblem {
    
	@Override
	public <V, E extends Comparable<E>> ArrayList<Edge<V>> mst(WeightedGraph<V, E> g) {
    ArrayList<Edge<V>> mstEdges = new ArrayList<>(); // O(1)
    Set<V> visited = new HashSet<>(); // O(1)

	// Bruker grafen sin comparator til å sortere edges
    // - Kompleksitet: O(1) - køen starter tom, og kantene legges til dynamisk.
    PriorityQueue<Edge<V>> edgeQueue = new PriorityQueue<>(g); // O(1)
	
	// Starter på en tilfeldig node.
    // // - Kompleksitet: O(1) for å velge startnoden og legge den til i "visited"-settet.
    V start = g.getFirstNode(); // O(1)
    visited.add(start); // O(1)

    // Legger så alle edges til denne noden i en priorityQueue
    // - Kompleksitet: O(deg(start) * log m), der deg(start) er antall noder tilknyttet start.
    // Hver kant settes inn i priority queue med O(log m), gjentatt for alle tilknyttede kanter.
    for (Edge<V> edge : g.adjacentEdges(start)) { // O(log m)
        edgeQueue.add(edge);
    }

    // Fortsetter til alle vertices er inkludert, eller det ikke er flere edges.
    // - Løkken kjører for hver kant i grafen maksimalt én gang (m kanter totalt).
    // - Hver operasjon med priority queue (poll eller add) tar O(log m).
    //   Derfor er den totale kompleksiteten for løkken O(m log m).
    while (!edgeQueue.isEmpty()) {

        // Henter den minste kanten fra priority queue:
        // - Kompleksitet: O(log m) for å fjerne kanten med lavest vekt.
        Edge<V> currentEdge = edgeQueue.poll();

        // Sjekker hvilken node som ikke er besøkt:
        // - Kompleksitet: O(1) for å sjekke medlemskap i HashSet.
        V destination = currentEdge.a;
        if (visited.contains(destination)) {
            destination = currentEdge.b;
        }

        // Hvis noden (destinasjonen) ikke er besøkt:
        // - Kompleksitet: O(1) for "contains"-sjekken og å legge til kanten/noden i MST.
        if (!visited.contains(destination)) {
            mstEdges.add(currentEdge);
            visited.add(destination);

            // Legger til kantene til den nye noden:
            // - Kompleksitet: O(deg(destination) * log m) for å legge til de nye kantene.
            //   Hver kant legges til priority queue med O(log m).
            for (Edge<V> edge : g.adjacentEdges(destination)) {

				// Unngår å legge til kanter der begge nodene er besøkt:
                // - Kompleksitet: O(1) for hver "visited"-sjekk.
                if (!visited.contains(edge.a) || !visited.contains(edge.b)) {
                    edgeQueue.add(edge);
                }
            }
        }
    }

    return mstEdges;
}


    @Override
    public <V> V lca(Graph<V> g, V root, V u, V v) {

        // Finner stien fra root til u og fra root til v
        // Kompleksitet: O(m + n) for hver "findPath"-kall, hvor m er antall kanter og n er antall noder i grafen.
        List<V> pathU = findPath(root, u, g);
        List<V> pathV = findPath(root, v, g);

        V lca = root; // Standardverdi er root hvis ingen annen felles forfar finnes
        int minLength = Math.min(pathU.size(), pathV.size());

        // Itererer gjennom begge stier for å finne siste felles forfar (LCA)
        // Kompleksitet: O(min(d_u, d_v)), hvor d_u og d_v er dybden til nodene u og v. 
        // Vi sammenligner hver node i begge stier til de divergerer.

        for (int i = 0; i < minLength; i++) {
            if (pathU.get(i).equals(pathV.get(i))) {
                lca = pathU.get(i); // Oppdaterer LCA til nåværende felles node
            } else {
                break; // Stiene divergerer her, så vi stopper løkken
            }
            }

        return lca; 
    }

	private <V> List<V> findPath(V root, V goal, Graph<V> graph) {
        Queue<V> queue = new LinkedList<>();
        Set<V> visited = new HashSet<>();
        Map<V, V> predecessor = new HashMap<>(); // To keep track of the path
    
        queue.add(root);
        visited.add(root);
    
        // Utfører BFS for å finne stien fra root til goal
        // Kompleksitet: O(m + n), der m er antall kanter og n er antall noder.
        while (!queue.isEmpty()) {
            V current = queue.poll();
    
            // Sjekker alle naboer til den nåværende noden
            // Kompleksitet: Hver nabo blir sjekket én gang totalt gjennom BFS, noe som gir O(m) i verste fall.
            for (V neighbor : graph.neighbours(current)) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    queue.add(neighbor);
                    predecessor.put(neighbor, current); 
    
                    // Hvis vi finner målnoden, rekonstruer stien
                    if (neighbor.equals(goal)) {
                        List<V> path = new ArrayList<>();
                        V step = goal;
    
                        // Rekonstruerer stien fra goal til root ved bruk av predecessor-kartet
                        // Kompleksitet: O(d_goal), der d_goal er dybden fra root til goal.
                        while (step != null) {
                            path.add(step);
                            step = predecessor.get(step);
                        }
                        
                        // Reverserer stien for å få den fra root til goal
                        Collections.reverse(path); 
                        return path;
                    }
                }
            }
        }
    
        // Return an empty list if no path is found from root to goal
        return new ArrayList<>();
    }
    
    @Override
public <V> Edge<V> addRedundant(Graph<V> g, V root) {

    // Sjekker om roten har kun én nabo
    // Kompleksitet: O(1), da vi sjekker størrelsen på listen over naboer
    V firstNode = root;
    V secondNode = root;
    boolean onlyOneRoot = false;
    if (g.neighbours(root).size() == 1) {
        onlyOneRoot = true;
    }

    // Lager en HashSet av rotens naboer
    // Kompleksitet: O(k), der k er grad av roten (antall naboer)
    HashSet<V> rootNeighbours = new HashSet<>();
    for (V neighbour : g.neighbours(root)) {
        rootNeighbours.add(neighbour);
    }

    // Finner det største undertreet ved å kalle biggestTree
    // Kompleksitet: O(m + n), hvor m er antall kanter og n er antall noder
    V rootBiggestSubTree = biggestTree(g, rootNeighbours, root);
    rootNeighbours.remove(rootBiggestSubTree);

    if (onlyOneRoot) {
        // Hvis roten har kun én nabo, finner vi den dypeste noden i det største undertreet
        // Kompleksitet: O(m + n)
        secondNode = findDeepestNodeInSubtree(g, rootBiggestSubTree, root);

    } else {
        // Hvis roten har flere naboer, finner vi de dypeste nodene i de to største undertrærne
        // Kompleksitet for hvert kall: O(m + n)
        firstNode = findDeepestNodeInSubtree(g, rootBiggestSubTree, root);
        V nextBiggestRoot = biggestTree(g, rootNeighbours, root);
        secondNode = findDeepestNodeInSubtree(g, nextBiggestRoot, root);
    }

    return new Edge<V>(firstNode, secondNode);
}
private <V> V biggestTree(Graph<V> g, HashSet<V> rootNeighbours, V originalroot) {

     // Initialiserer datastrukturer for å holde dybde og størrelsen på undertrær
    HashMap<V, Integer> depthMap = new HashMap<>();
    HashMap<V, Integer> subtreeSizeMap = new HashMap<>();
    V depthNode = null;

    // For hver nabo til roten, utfører vi BFS for å finne størrelsen på undertreet
    // Vi unngår å besøke noder som allerede er besøkt i tidligere BFS
    // Kompleksitet: Totalt O(m + n), siden hver node og kant besøkes maksimalt én gang
    for (V ver : rootNeighbours) {
        Queue<V> queue = new LinkedList<>();
        queue.add(ver);
        depthMap.put(ver, 0);
        int currentSubtreeSize = 0;

        while (!queue.isEmpty()) {
            V current = queue.poll();
            int currentDepth = depthMap.get(current);

            for (V neighbor : g.neighbours(current)) {
                if (!depthMap.containsKey(neighbor) && !neighbor.equals(originalroot)) {
                    depthMap.put(neighbor, currentDepth + 1);
                    queue.add(neighbor);
                    currentSubtreeSize++;
                }
            }
        }
        subtreeSizeMap.put(ver, currentSubtreeSize);
    }
    // Finner naboen med størst underliggende tre basert på størrelsen vi har beregnet
    // Kompleksitet: O(k), der k er antall naboer til roten
    int maxSubtreeSize = -1;
    for (Map.Entry<V, Integer> entry : subtreeSizeMap.entrySet()) {
        if (entry.getValue() > maxSubtreeSize) {
            maxSubtreeSize = entry.getValue();
            depthNode = entry.getKey();
        }
    }

    return depthNode;

}
private <V> V findDeepestNodeInSubtree(Graph<V> g, V rootNode, V originalRoot) {
    if (rootNode == null)
        return null;

    // Utfører BFS for å finne den dypeste noden i undertreet
    // Starter fra rootNode og unngår å besøke originalRoot
    // Kompleksitet: O(m + n), da vi traverserer alle noder og kanter i undertreet
    Queue<V> queue = new LinkedList<>();

        queue.add(rootNode);
        HashMap<V, Integer> depthMap = new HashMap<>();
        depthMap.put(rootNode, 0);
        V deepestNode = rootNode;
        int maxDepth = 0;
        depthMap.put(originalRoot, null);

        while (!queue.isEmpty()) {
            V current = queue.poll();
            int currentDepth = depthMap.get(current);

            for (V neighbor : g.neighbours(current)) {
                if (!depthMap.containsKey(neighbor)) {
                    depthMap.put(neighbor, currentDepth + 1);
                    queue.add(neighbor);
                    if (currentDepth + 1 > maxDepth) {
                        maxDepth = currentDepth + 1;
                        deepestNode = neighbor;
                    }
                }
            }
        }
        return deepestNode; // Returnerer den dypeste noden funnet i undertreet
    }
}
