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
    
    /**
     * Implementerer Prims algoritme for å finne minimum spenntre (MST) av en vektet graf.
     * @param g Grafen som MST skal finnes for
     * @return En liste av kanter som utgjør MST
     * Kompleksitet: O(m log n) eller O((m+n)*log n), der m er antall kanter og n er antall noder
     */
	@Override
	public <V, E extends Comparable<E>> ArrayList<Edge<V>> mst(WeightedGraph<V, E> g) {
    ArrayList<Edge<V>> mstEdges = new ArrayList<>(); // O(1)
    Set<V> visited = new HashSet<>(); // O(1)

	// Initialiserer en prioritetskø for kantene, som sorterer kantene etter vekt
    // Kompleksitet: O(1) 
    PriorityQueue<Edge<V>> edgeQueue = new PriorityQueue<>(g); // O(1)
	
	// Starter på en vilkårlig node.
    V start = g.getFirstNode(); // O(1)
    visited.add(start); // O(1)

    // Legger alle kantene til startnoden i prioritetskøen
    // Kompleksitet: O(deg(start) * log n), der deg(start) er graden til startnoden
    for (Edge<V> edge : g.adjacentEdges(start)) { 
        edgeQueue.add(edge); // O(log n) per kant
    }

    // Fortsetter til alle noder er inkludert, eller det ikke er flere kanter
    // Total kompleksitet for løkken: O((m log n), siden hver kant håndteres maksimalt to ganger
    while (!edgeQueue.isEmpty()) {

        // Henter den kanten med minst vekt
        Edge<V> currentEdge = edgeQueue.poll(); // O(log n)

        // Finner destinasjonsnoden som ikke er besøkt
        V destination = currentEdge.a;
        if (visited.contains(destination)) {
            destination = currentEdge.b;
        }

        // Hvis destinasjonsnoden ikke er besøkt
        if (!visited.contains(destination)) {
            mstEdges.add(currentEdge); // Legger kanten til i MST
            visited.add(destination); // Marker noden som besøkt

            // Legger til alle kantene fra destinasjonsnoden til prioritetskøen
            // Kompleksitet: O(deg(destination) * log n), der deg(destination) er graden til noden
            for (Edge<V> edge : g.adjacentEdges(destination)) {

				// Unngår å legge til kanter der begge nodene er besøkt:
                if (!visited.contains(edge.a) || !visited.contains(edge.b)) {
                    edgeQueue.add(edge); //  O(log n) per innsetting
                }
            }
        }
    }

    return mstEdges;
    }

    /**
     * Finner den laveste felles stamfaren (LCA) til to noder i et tre.
     * @param g Grafen (antatt å være et tre)
     * @param root Roten til treet
     * @param u Første node
     * @param v Andre node
     * @return Den laveste felles stamfaren til u og v
     * Kompleksitet: O(n)
     */
    @Override
    public <V> V lca(Graph<V> g, V root, V u, V v) {

        // Finner stien fra root til u og fra root til v
        // Kompleksitet: O(n) for hvert kall til findPath
        List<V> pathU = findPath(root, u, g);
        List<V> pathV = findPath(root, v, g);

        V lca = root; // Standardverdi er root hvis ingen annen felles forfar finnes
        int minLength = Math.min(pathU.size(), pathV.size());

        // Itererer gjennom begge stier for å finne siste felles forfar (LCA)
        // Kompleksitet: O(min(d_u, d_v)), hvor d_u og d_v er dybden til nodene u og v
        for (int i = 0; i < minLength; i++) {
            if (pathU.get(i).equals(pathV.get(i))) {
                lca = pathU.get(i); // Oppdaterer LCA til nåværende felles node
            } else {
                break; // Stiene divergerer her, så vi stopper løkken
            }
            }

        return lca; 
    }
    /**
     * Hjelpemetode som finner stien fra root til en gitt målnode ved bruk av BFS.
     * @param root Startnoden
     * @param goal Målnoden
     * @param graph Grafen
     * @return En liste over nodene på stien fra root til goal
     * Kompleksitet: O(n), der n er antall noder i treet
     */
	private <V> List<V> findPath(V root, V goal, Graph<V> graph) {
        Queue<V> queue = new LinkedList<>();
        Set<V> visited = new HashSet<>();
        Map<V, V> predecessor = new HashMap<>(); // For å holde styr på forløperen til hver node
    
        queue.add(root);
        visited.add(root);
    
        // Utfører BFS for å finne stien fra root til goal
        // Kompleksitet: O(n)
        while (!queue.isEmpty()) {
            V current = queue.poll();
    
            // Sjekker alle naboer til den nåværende noden
            // Hver nabo blir sjekket én gang totalt gjennom BFS
            for (V neighbor : graph.neighbours(current)) {
                if (!visited.contains(neighbor)) {
                    visited.add(neighbor);
                    queue.add(neighbor);
                    predecessor.put(neighbor, current); 
    
                    // Hvis vi finner målnoden, rekonstruerer vi stien
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

    /**
    * Finner to noder som skal kobles sammen med en ny kant for å minimere diameteren til treet.
    * @param g Grafen (antatt å være et tre)
    * @param root Roten til treet
    * @return En kant som kan legges til mellom to noder for å redusere diameteren
    * Kompleksitet: O(n), der n er antall noder.
    */
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
    // Kompleksitet: O(n), hvor n er antall noder
    V rootBiggestSubTree = biggestTree(g, rootNeighbours, root);
    rootNeighbours.remove(rootBiggestSubTree);

    if (onlyOneRoot) {
        // Hvis roten har kun én nabo, finner vi den dypeste noden i det største undertreet
        // Kompleksitet: O(n)
        secondNode = findDeepestNodeInSubtree(g, rootBiggestSubTree, root);

    } else {
        // Hvis roten har flere naboer, finner vi de dypeste nodene i de to største undertrærne
        // Kompleksitet for hvert kall: O(n)
        firstNode = findDeepestNodeInSubtree(g, rootBiggestSubTree, root);
        V nextBiggestRoot = biggestTree(g, rootNeighbours, root);
        secondNode = findDeepestNodeInSubtree(g, nextBiggestRoot, root);
    }

    return new Edge<V>(firstNode, secondNode);
}

/**
     * Finner den naboen til roten som har det største undertreet.
     * @param g Grafen
     * @param rootNeighbours Naboene til roten
     * @param originalRoot Roten til treet
     * @return Noden som er roten til det største undertreet
     * Kompleksitet: O(n)
     */
private <V> V biggestTree(Graph<V> g, HashSet<V> rootNeighbours, V originalroot) {

     // Initialiserer datastrukturer for å holde dybde og størrelsen på undertrær
    HashMap<V, Integer> depthMap = new HashMap<>();
    HashMap<V, Integer> subtreeSizeMap = new HashMap<>();
    V depthNode = null;

    // For hver nabo til roten, utfører vi BFS for å finne størrelsen på undertreet
    // Vi unngår å besøke noder som allerede er besøkt i tidligere BFS
    // Kompleksitet: Totalt O(n), siden hver node og kant besøkes maksimalt én gang
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

/**
     * Finner den dypeste noden i et gitt undertre.
     * @param g Grafen
     * @param rootNode Roten til undertreet
     * @param originalRoot Roten til hele treet (som skal unngås)
     * @return Den dypeste noden i undertreet
     * Kompleksitet: O(n)
     */
private <V> V findDeepestNodeInSubtree(Graph<V> g, V rootNode, V originalRoot) {
    if (rootNode == null)
        return null;

    // Utfører BFS for å finne den dypeste noden i undertreet
    // Starter fra rootNode og unngår å besøke originalRoot
    // Kompleksitet: O(n), da vi traverserer alle noder og kanter i undertreet
    Queue<V> queue = new LinkedList<>();

        queue.add(rootNode);
        HashMap<V, Integer> depthMap = new HashMap<>();
        depthMap.put(rootNode, 0);
        V deepestNode = rootNode;
        int maxDepth = 0;
        depthMap.put(originalRoot, null); // Marker originalRoot som besøkt

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
