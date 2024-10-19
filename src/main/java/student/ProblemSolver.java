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
public <V> Edge<V> addRedundant(Graph<V> tree, V root) {
    // Step 1: Identify all pairs of nodes that are not directly connected by an edge
    List<Edge<V>> candidateEdges = findAllCandidateEdges(tree);
    
    // Store the minimum outage and the corresponding best edge to add
    int minOutage = Integer.MAX_VALUE;
    Edge<V> bestEdge = null;

    // Step 2: Iterate through all candidate edges
    for (Edge<V> candidate : candidateEdges) {
        // Step 3: Create a modified graph with the candidate edge added
        Graph<V> modifiedTree = tree.clone();
        modifiedTree.addEdge(candidate.a, candidate.b);

        // Step 4: Find the worst case outage after removing one edge in the modified graph
        int worstOutage = getWorstOutage(modifiedTree, root, candidate);

        // Step 5: Update the best edge if the current candidate results in a smaller outage
        if (worstOutage < minOutage) {
            minOutage = worstOutage;
            bestEdge = candidate;
        }
    }

    return bestEdge;
}

// Helper function to calculate the worst outage after removing one edge
private <V> int getWorstOutage(Graph<V> graph, V root, Edge<V> addedEdge) {
    int maxOutage = 0;

    // Step 1: For every edge in the graph except the added edge, simulate its removal
    for (Edge<V> edge : graph.edges()) {
        // Skip the added edge since it's part of the cycle and doesn't cause an outage when removed
        if (edge.equals(addedEdge)) {
            continue;
        }

        // Step 2: Clone the graph and remove the current edge
        Graph<V> modifiedGraph = graph.clone();
        modifiedGraph.removeEdge(edge);

        // Step 3: Calculate the number of nodes disconnected from the root (outage)
        int outage = calculateOutage(modifiedGraph, root);
        
        // Track the worst outage
        maxOutage = Math.max(maxOutage, outage);
    }

    return maxOutage;
}

// Helper function to find all pairs of nodes that are not directly connected by an edge
private <V> List<Edge<V>> findAllCandidateEdges(Graph<V> tree) {
    List<Edge<V>> candidates = new ArrayList<>();
    Set<V> nodes = new HashSet<>();
    
    // Collect all nodes in the graph
    for (V node : tree.vertices()) {
        nodes.add(node);
    }

    // Iterate over all pairs of nodes and find missing edges
    for (V u : nodes) {
        for (V v : nodes) {
            // Add only if the nodes are not equal and there is no existing edge between them
            if (!u.equals(v) && !tree.adjacent(u, v)) {
                candidates.add(new Edge<>(u, v));
            }
        }
    }

    return candidates;
}

// Helper function to calculate the size of the outage (number of disconnected nodes)
private <V> int calculateOutage(Graph<V> graph, V root) {
    Set<V> visited = new HashSet<>();
    dfs(graph, root, visited);

    // All nodes that are not visited after DFS are considered disconnected
    return graph.size() - visited.size();
}

// Standard DFS implementation to explore the connected component from root
private <V> void dfs(Graph<V> graph, V node, Set<V> visited) {
    visited.add(node);
    for (V neighbor : graph.neighbours(node)) {
        if (!visited.contains(neighbor)) {
            dfs(graph, neighbor, visited);
        }
    }
}

    
    class UnionFind<V> {
        private final Map<V, V> parent = new HashMap<>();
        private final Map<V, Integer> rank = new HashMap<>();
        private final Map<V, Integer> size = new HashMap<>();
    
        public void add(V v) {
            parent.put(v, v);
            rank.put(v, 0);
            size.put(v, 1);
        }
    
        public V find(V v) {
            if (parent.get(v) != v) {
                parent.put(v, find(parent.get(v))); // Path compression
            }
            return parent.get(v);
        }
    
        public void union(V a, V b) {
            V rootA = find(a);
            V rootB = find(b);
    
            if (rootA != rootB) {
                // Union by rank
                if (rank.get(rootA) > rank.get(rootB)) {
                    parent.put(rootB, rootA);
                    size.put(rootA, size.get(rootA) + size.get(rootB));
                } else if (rank.get(rootA) < rank.get(rootB)) {
                    parent.put(rootA, rootB);
                    size.put(rootB, size.get(rootA) + size.get(rootB));
                } else {
                    parent.put(rootB, rootA);
                    size.put(rootA, size.get(rootA) + size.get(rootB));
                    rank.put(rootA, rank.get(rootA) + 1);
                }
            }
        }
    
        public void undoUnion(V a, V b) {
            // This method restores the original parent and size of the union
            V rootA = find(a);
            V rootB = find(b);
            
            if (rootA == b) {
                parent.put(rootA, rootA);
                size.put(rootA, size.get(rootA) - size.get(rootB));
            }
        }
    
        public int getSize(V v) {
            return size.get(find(v));
        }
    }}