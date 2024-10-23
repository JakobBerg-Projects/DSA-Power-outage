# Answer File - Semester 2
# Description of each Implementation
Briefly describe your implementation of the different methods. What was your idea and how did you execute it? If there were any problems and/or failed implementations please add a description.

## Task 1 - mst
I min implementasjon av Prims algoritme for å finne minimum spenntre (MST) starter jeg med å opprette tre datastrukturer: mstEdges, visited og edgeQueue. mstEdges er en liste som skal inneholde kantene som til slutt utgjør MST. visited er en mengde som holder styr på hvilke noder som allerede er inkludert i MST, og edgeQueue er en prioritetskø som organiserer kantene basert på deres vekt, slik at den med lavest vekt alltid blir behandlet først.

Algoritmen begynner ved å velge en vilkårlig startnode, som legges til i visited. Deretter legges alle kantene som er tilknyttet startnoden inn i edgeQueue. Hovedideen er å alltid velge den kanten som forbinder en allerede besøkt node med en ikke-besøkt node, og som har den minste vekten blant slike kanter.

Så lenge ikke alle noder er besøkt og edgeQueue ikke er tom, fortsetter algoritmen å fjerne den kanten med lavest vekt fra køen. For hver kant undersøkes begge endenoder for å finne ut hvilken som ikke er besøkt. Hvis en slik node finnes, legges kanten til mstEdges, og noden markeres som besøkt ved å legge den til i visited. Deretter legges alle kantene som er tilknyttet den nylig besøkte noden, og som fører til ikke-besøkte noder, inn i edgeQueue.

Denne prosessen fortsetter til alle noder er inkludert i MST eller edgeQueue er tom. Ved å alltid velge den minste tilgjengelige kanten sikrer vi at den resulterende trestrukturen har minimal total vekt.xw

## Task 2 - lca
For å finne den laveste felles stamfaren (LCA) til to noder i et tre, implementerte jeg først en hjelpefunksjon findPath. Denne funksjonen bruker bredde-først-søk (BFS) for å finne stien fra roten til en gitt målnode. Ved å holde styr på forgjengerne til hver besøkt node, kan vi rekonstruere stien tilbake til roten når vi har funnet målnoden.

I hovedfunksjonen lca kaller jeg findPath to ganger for å finne stiene fra roten til henholdsvis node u og node v. Disse stiene lagres i to lister, pathU og pathV. Deretter itererer jeg gjennom begge listene samtidig, starter fra roten, og sammenligner nodene på hver indeks. Så lenge nodene er like, fortsetter jeg å iterere. Den siste noden som er lik i begge stier før de divergerer, er den laveste felles stamfaren. Hvis stiene divergerer tidligere, returnerer jeg den forrige noden som LCA.

Denne metoden er effektiv fordi den utnytter treets egenskaper og trenger kun å traversere fra roten til de to aktuelle nodene.

## Task 3 - addRedundant
Målet med denne metoden er å finne to noder i treet slik at ved å legge til en kant mellom dem, minimerer vi diameteren til treet. Først sjekker jeg om roten har kun én nabo. Hvis dette er tilfellet, er treet i hovedsak en linje, og vi trenger bare å finne den dypeste noden i det eneste undertreet.

Hvis roten har flere naboer, bruker jeg funksjonen biggestTree for å finne det største undertreet blant rotens naboer. Dette gjøres ved å utføre BFS fra hver av rotens naboer og telle antall noder i hvert undertre. Den naboen med flest noder anses å være roten til det største undertreet.

Deretter finner jeg den dypeste noden i det største undertreet ved hjelp av findDeepestNodeInSubtree. Hvis roten har flere enn én nabo, finner jeg også den nest største naboen og den dypeste noden i det tilhørende undertreet. Til slutt returnerer jeg en ny kant mellom de to dypeste nodene funnet. Ved å koble sammen de dypeste nodene i de to største undertrærne, reduserer vi treets diameter mest mulig.


# Runtime Analysis
For each method of the different strategies give a runtime analysis in Big-O notation and a description of why it has this runtime.

**If you have implemented any helper methods you must add these as well.**

* ``mst(WeightedGraph<T, E> g)``: O(m log n)
    Kjøretidsanalysen for denne metoden baserer seg på antall operasjoner som utføres i forhold til antall noder n og antall kanter m i grafen.

    Oppstart:
    Opprettelsen av datastrukturer mstEdges, visited og edgeQueue tar konstant tid, altså O(1).
    Legge til startnoden i visited er også en O(1) operasjon.
    Å legge til alle kantene fra startnoden til edgeQueue tar O(deg(start) * log n) tid, hvor deg(start) er graden til startnoden. I verste fall kan dette være opptil O(n log n), men siden dette ikke dominerer total kompleksitet, kan det ignoreres i den asymptotiske analysen.
    
    Hovedløkken:
    Løkken fortsetter så lenge edgeQueue ikke er tom og ikke alle noder er besøkt.
    Hver gang vi legger til eller fjerner en kant fra edgeQueue, tar dette O(log n) tid.
    Siden hver kant i grafen kan legges til køen maksimalt én gang (i begge retninger hvis grafen er urettet), er antall køoperasjoner proporsjonalt med antall kanter, altså O(m).

    Derfor er den totale tiden brukt på køoperasjoner O(m log n).

    Konklusjon:
    Den dominerende faktoren i algoritmens kjøretid er køoperasjonene, som gir en total kompleksitet på O(m log n).

* ``lca(Graph<T> g, T root, T u, T v)``: O(n)
    findPath-funksjonen:
    Utfører BFS for å finne stien fra roten til en målnode.
    I et tre med n noder vil BFS besøke hver node én gang, noe som gir en kompleksitet på O(n).
    Rekonstruksjon av stien fra målnoden tilbake til roten tar også maksimalt O(n) tid.

    Hovedfunksjonen lca:
    Kaller findPath to ganger, én for hver av nodene u og v. Dette gir en samlet kompleksitet på O(n) + O(n) = O(n).
    Sammenligningen av de to stiene gjøres ved å iterere gjennom listene til den korteste av stiene. Dette tar O(n) tid i verste fall.

    Konklusjon:
    Total kompleksitet for lca-metoden er O(n), da alle operasjonene er lineære i forhold til antall noder.

* ``addRedundant(Graph<T> g, T root)``: O(n)
    Hovedmetoden:
    Sjekker om roten har kun én nabo, som tar O(1) tid.
    Kopierer rotens naboer inn i en HashSet, som i verste fall tar O(n) tid dersom roten er koblet til alle andre noder.
    Kaller biggestTree for å finne det største undertreet, som har kompleksitet O(n).
    Avhengig av om roten har én eller flere naboer, kalles findDeepestNodeInSubtree én eller to ganger, hver med kompleksitet O(n).

    biggestTree-funksjonen:
    Utfører BFS fra hver av rotens naboer for å beregne størrelsen på deres undertrær.
    Ved å bruke en depthMap unngår vi å besøke samme node flere ganger.
    Total kompleksitet er O(n), da hver node i treet blir besøkt maksimalt én gang over alle BFS-kallene.

    findDeepestNodeInSubtree-funksjonen:
    Utfører BFS for å finne den dypeste noden i et gitt undertre.
    Kompleksiteten er O(n), siden vi i verste fall kan besøke alle noder i treet.
    
    Konklusjon:
    Selv om vi har flere kall til funksjoner med kompleksitet O(n), er antallet kall konstant (ikke avhengig av n), så total kompleksitet for addRedundant er fortsatt O(n).