/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for implementation of graph search algorithms.

    License: Michigan Honor License

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
    var newIdx = heap.length;
    var parentIdx = Math.floor((newIdx - 1)/2);

    heap.push(new_element);

    var heaped = (newIdx <= 0) || heap[parentIdx] <= heap[newIdx];

    while(!heaped) {
        var temp = heap[parentIdx];
        heap[parentIdx] = heap[newIdx];
        heap[newIdx] = temp;

        newIdx = parentIdx;
        parentIdx = Math.floor((newIdx - 1)/2)

        heaped = (newIdx <= 0) || heap[parentIdx].priority <= heap[newIdx].priority;
    }
    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
    var lastIdx = heap.length - 1;
    var minElement = heap[0];
    [heap[0], heap[lastIdx]] = [heap[lastIdx], heap[0]];
    heap.pop();
    var k = 0;

    while (2*k + 1 < heap.length) {
        var j = 2*k + 1; // start with right child
        if (j + 1< heap.length && heap[j+1].priority < heap[j].priority) ++j;
        if (heap[k].priority <= heap[j].priority) break;
        [heap[j], heap[k]] = [heap[k], heap[j]];
        k = j
    }

    return minElement;
    // STENCIL: implement your min binary heap extract operation
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract
    // STENCIL: ensure extract method is within minheaper object

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };
            if (Math.abs(G[iind][jind].x- q_init[0])<=0.5*eps && Math.abs(G[iind][jind].y- q_init[1])<=0.5*eps) {
                G[iind][jind].queued = true;
                G[iind][jind].start = 0;
                G[iind][jind].distance = 0;
                minheap_insert(visit_queue, G[iind][jind]);
            }
            // STENCIL: determine whether this graph node should be the start
            //   point for the search
        }
    }
}

function iterateGraphSearch() {
    coord = [[1, 0], [0, -1], [-1, 0], [0, 1]];
    node = minheap_extract(visit_queue);
    xpos = node.x;
    ypos = node.y;
    draw_2D_configuration([node.x, node.y], "visited"); 

    for (let i = 0; i < 4; i++) {
        ii = node.i + coord[i][0];
        jj = node.j + coord[i][1];
        if (!G[ii][jj].visited && !testCollision([G[ii][jj].x, G[ii][jj].y])) {
            if (Math.abs(q_goal[0] - xpos) <= 0.5*eps &&  Math.abs(q_goal[1] - ypos) <= 0.5*eps) {
                search_iterate = false;
                console.log(G[ii][jj].x);
                G[ii][jj].parent = node;
                drawHighlightedPathGraph(G[ii][jj]);
                return "succeeded";
            }
            else if (G[ii][jj].distance > node.distance + getDistance(xpos, G[ii][jj].x, ypos, G[ii][jj].y)) {
                G[ii][jj].parent = node;
                G[ii][jj].queued = true;
                G[ii][jj].distance = node.distance + getDistance(xpos, G[ii][jj].x, ypos, G[ii][jj].y);
                G[ii][jj].priority = node.distance + getDistance(xpos, G[ii][jj].x, ypos, G[ii][jj].y) + getDistance(q_goal[0], G[ii][jj].x, q_goal[1], G[ii][jj].y);
                minheap_insert(visit_queue, G[ii][jj]);
                draw_2D_configuration([G[ii][jj].x, G[ii][jj].y], "queued"); 
            }
        }
    }
    if (visit_queue.length==0) {
        search_iterate = false;
        return "failed";
    }
    return "iterating";
    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.

function getDistance(x1, x2, y1, y2) {
    return Math.sqrt(Math.pow(x1-x2, 2) + Math.pow(y1-y2, 2));
}