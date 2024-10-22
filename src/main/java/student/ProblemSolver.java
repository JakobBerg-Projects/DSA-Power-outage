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
      // Step 1: Precompute the subtree sizes for each node
      Map<V, Integer> subtreeSizes = calculateSubtreeSizes(tree, root);
        
      // Step 2: Find all potential candidate edges that are missing in the tree
      List<Edge<V>> candidateEdges = findAllCandidateEdges(tree);

      // Initialize variables to store the best edge and minimum outage
      int minWorstOutage = Integer.MAX_VALUE;
      Edge<V> bestEdge = null;

      // Step 3: Iterate through all candidate edges and evaluate them
      for (Edge<V> candidate : candidateEdges) {
          // Create a clone of the tree and add the candidate edge to form a cycle
          Graph<V> modifiedTree = tree.clone();
          modifiedTree.addEdge(candidate.a, candidate.b);

          // Step 4: Calculate the worst-case outage after removing one edge from the modified graph
          int worstOutage = calculateWorstOutage(modifiedTree, root, subtreeSizes);

          // Update the best edge if this candidate provides a smaller outage
          if (worstOutage < minWorstOutage) {
              minWorstOutage = worstOutage;
              bestEdge = candidate;
          }
      }

      return bestEdge;
  }

  /**
   * Precompute the size of the subtree rooted at each node.
   */
  private <V> Map<V, Integer> calculateSubtreeSizes(Graph<V> tree, V root) {
      Map<V, Integer> subtreeSizes = new HashMap<>();
      calculateSubtreeSizeDFS(tree, root, null, subtreeSizes);
      return subtreeSizes;
  }

  /**
   * DFS to calculate the size of the subtree rooted at a node.
   */
  private <V> int calculateSubtreeSizeDFS(Graph<V> tree, V current, V parent, Map<V, Integer> subtreeSizes) {
      int size = 1; // The current node counts as 1
      for (V neighbor : tree.neighbours(current)) {
          if (!neighbor.equals(parent)) {
              size += calculateSubtreeSizeDFS(tree, neighbor, current, subtreeSizes);
          }
      }
      subtreeSizes.put(current, size);
      return size;
  }

  /**
   * Find all possible pairs of nodes that are not directly connected by an edge.
   */
  private <V> List<Edge<V>> findAllCandidateEdges(Graph<V> tree) {
      List<Edge<V>> candidateEdges = new ArrayList<>();
      Set<V> vertices = new HashSet<>();
      
      // Collect all vertices
      for (V vertex : tree.vertices()) {
          vertices.add(vertex);
      }

      // Check all pairs of vertices and find missing edges
      for (V u : vertices) {
          for (V v : vertices) {
              if (!u.equals(v) && !tree.adjacent(u, v)) {
                  candidateEdges.add(new Edge<>(u, v));
              }
          }
      }

      return candidateEdges;
  }

  /**
   * Calculate the worst-case outage after removing any edge from the graph.
   * For each edge removal, the worst-case outage is the number of nodes that are disconnected from the root.
   */
  private <V> int calculateWorstOutage(Graph<V> modifiedTree, V root, Map<V, Integer> subtreeSizes) {
      int maxOutage = 0;

      // Step 1: Iterate over all edges in the modified graph (except the newly added edge)
      for (Edge<V> edge : modifiedTree.edges()) {
          // Simulate removing this edge from the graph
          modifiedTree.removeEdge(edge);

          // Step 2: Use DFS to calculate the size of the disconnected components
          Set<V> visited = new HashSet<>();
          dfs(modifiedTree, root, visited); // Perform DFS starting from the root
          
          // Calculate the number of disconnected nodes (i.e., the size of the largest disconnected component)
          int disconnectedNodes = modifiedTree.size() - visited.size();

          // Step 3: Restore the edge back to the modified tree after simulating its removal
          modifiedTree.addEdge(edge);

          // Step 4: Track the worst outage (i.e., the largest number of disconnected nodes after edge removal)
          maxOutage = Math.max(maxOutage, disconnectedNodes);
      }

      return maxOutage;
  }

  /**
   * Standard DFS implementation to explore the connected component from the root.
   */
  private <V> void dfs(Graph<V> graph, V current, Set<V> visited) {
      visited.add(current);
      for (V neighbor : graph.neighbours(current)) {
          if (!visited.contains(neighbor)) {
              dfs(graph, neighbor, visited);
          }
      }
  }
}