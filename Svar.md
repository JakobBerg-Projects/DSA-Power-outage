# Answer File - Semester 2
# Description of each Implementation
Briefly describe your implementation of the different methods. What was your idea and how did you execute it? If there were any problems and/or failed implementations please add a description.

## Task 1 - mst
Start by creating 3 datastructues
mstEdges: This list will store the edges that form the Minimum Spanning Tree.
visited: A set to track which vertices have been visited.
edgeQueue: A priority queue (min-heap) to store edges based on their weights, ensuring that the smallest edge is processed first.

Start on a random node which we will add to visited.
We will then add all the edges from this node to a priorityqueue. 

The algorithm continues until all vertices are included or there are no more edges to process.
The edge with the smallest weight is removed from the priority queue.

For the current edge, you check which vertex (either a or b) has not been visited. If one of them hasn't been visited yet, that vertex is chosen as the next vertex to explore.

If the selected vertex is unvisited, you add the edge to mstEdges.
The newly visited vertex is added to the visited set.
After adding the vertex, you also add all the edges connected to the newly visited vertex to the priority queue. However, you avoid adding edges where both vertices have already been visited to prevent cycling back to previously visited nodes.

The algorithm repeats the process of picking the smallest edge from the queue, visiting unvisited vertices, and adding new edges until all vertices are part of the MST or no more edges are left in the priority queue.

## Task 2 - lca
We start by generating a helper method which gives the path from root to vertex u and v
These paths are then stored in a list for each vertex. I then iterate over both route simultaniously. If current vertexes in both routes are the same, we continue in the loop.
However if they are not the same, then the previous vertex should be returned. 

## Task 3 - addRedundant
*Enter description*


# Runtime Analysis
For each method of the different strategies give a runtime analysis in Big-O notation and a description of why it has this runtime.

**If you have implemented any helper methods you must add these as well.**

* ``mst(WeightedGraph<T, E> g)``: O(m log m + n log m)
    Hver kant legges til og fjernes fra priority queue maksimalt én gang, som tar O(log m) per operasjon.
    Den totale kompleksiteten for hele metoden er derfor O(m log m + n log m), der m er antall kanter og n er antall noder.
    Der n log m kommer fra håndtering av noder, og m log m kommer fra håndtering av kanter.

* ``lca(Graph<T> g, T root, T u, T v)``: O(m + n + min(d_u, d_v))
    findPath-metoden:
    Tidskompleksitet: O(m + n), hvor m er antall kanter og n er antall noder i grafen.
    Dette skyldes at BFS besøker hver node én gang og sjekker alle tilknyttede kanter.

    lca-metoden:
    Tidskompleksitet: O(m + n + min(d_u, d_v)), hvor m er antall kanter, n er antall noder, og d_u og d_v er dybden til nodene u og v.
    Dette inkluderer to kall til findPath, som hver er O(m + n), og deretter en O(min(d_u, d_v)) løkke for å finne siste felles forfar.

* ``addRedundant(Graph<T> g, T root)``: O(?)
    * *Insert description of why the method has the given runtime*

