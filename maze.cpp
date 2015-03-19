#include <fstream>
#include <assert.h>
#include <sstream>
#include <iostream>
#include "maze.h"
#include <iomanip>
#include <sys/resource.h>
#include <math.h>
#include "graphics.h"
#include <cstdlib> 
#include <cstdio> 
#include <vector>

#include <queue>
#include <map>
#include <iterator>

using namespace std;

int **G;                    //the whole graph
Point* upperright;          //the upper right point of the graph
Path terms;                 //terminals to be connected
ObstacleVector obs;         //obstacles
PathVector completeGraph;   
PathVector mst;             //minimum spanning tree for Prim's algorithm
PathVector mst_k;           //minimum spanning tree for Kruskal's algorithm

Path **p = NULL;
int **disRec;
Point ***pt;

void dump_to_file(char* fn) {
    ofstream myfile;
    myfile.open(fn);
    int len = 0;
    for (int i = 1; i <= upperright->x; i++)
        for (int j = 1; j <= upperright->y; j++)
            if(G[i][j] == 1)
                len++;
    for (int j = upperright->y; j > 0; j--) {
        for (int i = 1; i <= upperright->x; i++) {
            if(G[i][j] == MAX)
                myfile << " N | ";
            else
                myfile << setw(2) << G[i][j] << " | ";
        }
        myfile << endl;
    }
    myfile << "Total wirelength:" << len << endl;
    myfile.close();
}

int main(int argc, char *argv[]) {

    double beginTime = mazegetTime();
    
    cout << "Begin ..." << endl;
    init(argv[1]);

    //part 1: finish prim() of MST
    prim();

    //print out MST
    
    cout << "Printing out MST ..." << endl;
    for (PathVector::iterator spi = mst.begin(); spi != mst.end(); spi++) {
        Path* sp = (*spi);
        for (Path::iterator pi = sp->begin(); pi != sp->end(); pi++) {
            G[(*pi)->x][(*pi)->y] = 1;
        }
    }
    //calculate wirelength
    int len = 0;
    for (int i = 1; i <= upperright->x; i++)
        for (int j = 1; j <= upperright->y; j++)
            if(G[i][j] == 1)
                len++;
   
    print_graph();
    // write function to dump you results into a file called 
    // "mst_result.txt"
    // the content is the same as the printed content in print_graph();

    char* file_name = "mst_result.txt";
    dump_to_file(file_name);
    clean();
    
    // part 2: design your own routing algorithm
    cout << "Printing out Kruskal's implement result ..." << endl;
    kruskal();
    for (PathVector::iterator spi = mst_k.begin(); spi != mst_k.end(); spi++) {
        Path* sp = (*spi);
        for (Path::iterator pi = sp->begin(); pi != sp->end(); pi++) {
            G[(*pi)->x][(*pi)->y] = 1;
        }
    }
    print_graph();
    //dump results to file
    file_name = "routing_result.txt";
    dump_to_file(file_name);
    clean();

    //print out complete graph with every path whether adopted or not
    
    for (int i = 0; i < terms.size(); ++i)
        for (int j = i + 1; j < terms.size(); ++j)
            completeGraph.push_back(&p[i][j]);
    cout << "Printing out complete graph ..." << endl;
    for (PathVector::iterator spi = completeGraph.begin(); spi != completeGraph.end(); spi++) {
        Path* sp = (*spi);
        for (Path::iterator pi = sp->begin(); pi != sp->end(); pi++) {
            G[(*pi)->x][(*pi)->y] = 1;
        }
    }
    print_graph();
    clean();

    cout << "Total wirelength:" << len << endl;
    cout << "Total runtime:" << mazegetTime() - beginTime << " usec." << endl;

    cout << "Finished." << endl;
}

void init(char *fn) {
    ifstream in;
    in.open(fn);
    if (!in.is_open()) {
        cerr << "Could not open file " << fn << endl;
        assert(0);
    }
    string line, word;

    while(true) {
        getline(in, line);
        if (in.eof()) break;
        if (line.length() < 2) continue; // empty line, skip
        if (line.c_str()[0] == '#') continue;

        istringstream is(line);
        // get token
        is >> word;
        if(!word.compare("grid")) {
            is >> word;
            upperright = new Point();
            sscanf(word.c_str(), "(%d,%d)", &upperright->x, &upperright->y);
            cout << "grid (" << upperright->x << "," << upperright->y << ")" << endl;
        }
        if(!word.compare("term")) {
            is >> word;
            Point* tp = new Point();
            sscanf(word.c_str(), "(%d,%d)", &tp->x, &tp->y);
            cout << "term (" << tp->x << "," << tp->y << ")" << endl;
            terms.push_back(tp);
        }
        if(!word.compare("obs")) {
            is >> word;
            Point* tp1 = new Point();
            Point* tp2 = new Point();
            sscanf(word.c_str(), "(%d,%d)", &tp1->x, &tp1->y);
            cout << "obs (" << tp1->x << "," << tp1->y << ")";
            is >> word;
            sscanf(word.c_str(), "(%d,%d)", &tp2->x, &tp2->y);
            cout << " (" << tp2->x << "," << tp2->y << ")" << endl;
            Obstacle* to = new Obstacle(tp1,tp2);
            obs.push_back(to);
        }

    }
    in.close();

    G = new int*[upperright->x + 2];
    //set boundary
    for (int i = 0; i < upperright->x + 2; i++) {
        G[i] = new int[upperright->y + 2];
    }

    clean();
    
    for ( int i = 0; i < upperright->x + 2; i++) {
        G[i][0] = -1;
        G[i][upperright->y + 1] = -1;
    }

    for ( int i = 0; i < upperright->y + 2; i++) {
        G[0][i] = -1;
        G[upperright->x + 1][i] = -1;
    }

    //set obstacles
    for(ObstacleVector::iterator oi = obs.begin(); oi != obs.end();
        oi++) {
        for (int i = (*oi)->ll->x; i <= (*oi)->ur->x; i++)
            for (int j = (*oi)->ll->y; j <= (*oi)->ur->y; j++)
                G[i][j] = -1;

    }

    for(Path::iterator tp = terms.begin(); tp != terms.end(); tp++) {
            (*tp)->paths = new PathVector();
    }

    buildGraph();
}


void print_path(Path* path) {
    for (Path::iterator pi = path->begin(); pi != path->end(); pi++) {
        cout << "(" << (*pi)->x << "," << (*pi)->y << ") " << endl;
    }
}

void print_graph() {
    for (int j = upperright->y; j > 0; j--) {
        for (int i = 1; i <= upperright->x; i++) {
            if(G[i][j] == MAX)
                cout << " N | ";
            else
                cout << setw(2) << G[i][j] << " | ";
        }
        cout << endl;
    }

}

void clean() {
    for (int i = 1; i <= upperright->x; i++)
        for (int j = 1; j <= upperright->y; j++)
            if (G[i][j] != -1)
                G[i][j] = MAX;
}

//########

Path* maze(Point* s, Point* t);

Path* retrace(Point* s, Point* t);

void bfs(Point* s, int** G) {
    Point* prev[upperright->x + 2][upperright->y + 2];  //used to backtrace
    queue<Point*> q;
    G[s->x][s->y] = 0;
    Point* start = pt[s->x][s->y];
    q.push(start);
    while (!q.empty()) {
        Point* cur = q.front();             //current point to consider
        q.pop();
        int x[4] = {1, 0, -1, 0};
        int y[4] = {0, 1, 0, -1};
        for (int dir = 0; dir < 4; ++dir) {
            int temp = G[cur->x + x[dir]][cur->y + y[dir]];
            if (temp == MAX) {              //this point has not been visited
                q.push(pt[cur->x + x[dir]][cur->y + y[dir]]);
                G[cur->x + x[dir]][cur->y + y[dir]] = G[cur->x][cur->y] + 1;
                prev[cur->x + x[dir]][cur->y + y[dir]] = cur;   //record the previous point of the next point
            }
        }
    }
    int now = 0;            //now is the index of Point s in terms
    Path::iterator itr = terms.begin();
    for (; *itr != s; ++itr, ++now);
    int count = now;        //count is the index of the destination point starting from s
    for (; itr != terms.end(); ++itr, ++count) {
        disRec[now][count] = disRec[count][now] = G[(*itr)->x][(*itr)->y];
        Point* cur = pt[(*itr)->x][(*itr)->y];
        for (int i = 0; i < G[(*itr)->x][(*itr)->y]; ++i) {
            p[now][count].push_back(cur);
            cur = prev[cur->x][cur->y];
        }
        assert(cur->x == s->x && cur->y == s->y);   //for testing
        p[now][count].push_back(cur);
        p[count][now] = p[now][count];
    }
    clean();
}

void buildGraph() {
    //initialization for graph construction
    p = new Path*[terms.size()];
    disRec = new int*[terms.size()];
    pt = new Point**[upperright->x + 2];
    for (int i = 0; i < terms.size(); ++i) {
        p[i] = new Path[terms.size()];
        disRec[i] = new int[terms.size()];
    }
    for (int i = 0; i < upperright->x + 2; ++i) {
        pt[i] = new Point*[upperright->y + 2];
        for (int j = 0; j < upperright->y + 2; ++j)
            pt[i][j] = new Point(i, j);
    }
    cout << "Point matrix created." << endl;
    //get distances between every two nodes
    int now = 0;
    for (Path::iterator tItr = terms.begin(); tItr != terms.end(); ++tItr, ++now) {
        bfs(*tItr, G);
    }
    //cout << "Graph constructed." << endl;
}

Path::iterator find(Path::iterator s, Path::iterator e, Point* val) {
    Path::iterator temp = s;
    for (; temp != e; ++temp)
        if ((*s)->x == val->x && (*s)->y == val->y)
            break;
    return temp;
}

template <class T>
T next(T s, int num) {
    T temp = s;
    for (int i = 0; i < num; ++i, ++temp);
    return temp;
}

template <class T>
T prev(T s, int num) {
    T temp = s;
    for (int i = 0; i < num; ++i, --temp);
    return temp;
}
/*
void opt(PathVector pv){
    PathVector::iterator itr;
    bool done = false;
    while (!done) {
        done = true;
        for (itr = pv.begin(); itr != pv.end(); ++itr) {
            PathVector::iterator comp = next(itr, 1);
            for (; comp != pv.end(); ++comp) {          //examine every path afterwards in the MST
                bool meet = false;
                Path::iterator s = (*itr)->begin();
                Path::iterator e = (*itr)->end();
                for (Path::iterator pItr = (*comp)->begin(); pItr != (*comp)->end(); pItr++) {
                    Point *pt = *pItr;
                    if (meet) {
                        e = find((*itr)->begin(), (*itr)->end(), pt);
                        if (e != (*itr)->end()) {            //finds the second intersection
                            if (e == next(s, 1) || e == prev(s, 1)) {      //two adjacent points
                                s = e;
                                e = (*itr)->end();
                            } else {                    //finds the unnecessary segment
                                break;
                            }
                        }
                    } else {
                        s = find((*itr)->begin(), (*itr)->end(), pt);
                        if (s != (*itr)->end()) {            //finds the first intersection
                            meet = true;
                            continue;
                        }
                        else
                            s = (*itr)->begin();
                    }
                }
                if (e != (*itr)->end()) {                  //there is unnecessary segment of path
                    Path includedTerms;
                    if (s > e) {
                        Path::iterator temp = s;
                        s = e;
                        e = temp;
                    }
                    for (Path::iterator t = terms.begin(); t != terms.end(); ++t) { //check if there are other terminals in this path.
                        for (Path::iterator temp = next(s, 1); temp != e; ++temp) { //it will be ok to erase this part if it contains other terminals,
                            if ((*temp)->x == (*t)->x && (*temp)->y == (*t)->y) {   //but when we are considering this terminal once stands in the path,
                                includedTerms.push_back(*temp);                     //it is probably that part of the erased partial path will be adopted again.
                            }
                        }
                    }
                    if (includedTerms.empty()) {            //there is no other terminals in this path
                        (*itr)->erase(next(s, 1), e);
                        done = false;
                    }
                    else {
                        Path::iterator s2 = find((*comp)->begin(), (*comp)->end(), *s);
                        Path::iterator e2 = find((*comp)->begin(), (*comp)->end(), *e);
                        if (s2 > e2) {
                            Path::iterator temp = s2;
                            s2 = e2;
                            e2 = temp;
                        }
                        Path includedTerms2;
                        for (Path::iterator t = terms.begin(); t != terms.end(); ++t) {
                            for (Path::iterator temp = next(s2, 1); temp != e2; ++temp) {
                                if ((*temp)->x == (*t)->x && (*temp)->y == (*t)->y) {
                                    includedTerms2.push_back(*temp);
                                }
                            }
                        }
                        if (includedTerms2.empty()) {
                            (*comp)->erase(next(s2, 1), e);
                            done = false;
                        }
                        else {  //If both paths contain some other terminals, the situation will be complicated, and we are not given enough time to test out the code below.
                            int da = distance(s, find(s, e, *(includedTerms.begin())));
                            int db = distance(find(s, e, *(prev(includedTerms.end(), 1))), e);
                            int dc = distance(s2, find(s2, e2, *(includedTerms2.begin())));
                            int dd = distance(find(s2, e2, *(prev(includedTerms2.end(), 1))), e2);
                            int tbd = max(da, max(db, max(dc, dd))); 
                            if (tbd == da)
                                (*itr)->erase(next(s, 1), find(s, e, *(includedTerms.begin())));
                            else if (tbd == db)
                                (*itr)->erase(next(find(s, e, *(prev(includedTerms.end(), 1))), 1), e);
                            else if (tbd == dc)
                                (*itr)->erase(next(s2, 1), find(s2, e2, *(includedTerms2.begin())));
                            else
                                (*itr)->erase(next(find(s2, e2, *(prev(includedTerms2.end(), 1))), 1), e2);
                        }
                        
                    }
                }
            }
        }
    }
}
*/
void prim(){
    //use Prim's algorithm to find MST
    multimap<int, pair<int, int> > pool;        //contains paths not excluded
    for (int i = 0; i < terms.size(); ++i)
        for (int j = i + 1; j < terms.size(); ++j){
            pool.insert(pair<int, pair<int, int> >(disRec[i][j], pair<int, int>(i, j)));
        }
    //cout << "Candidate pool constructed." << endl;
    int linked[terms.size()];                //keeps record of terminals already adopted
    for (int i = 0; i < terms.size(); ++i)
        linked[i] = 0;                      //initialization. None has been adopted
    multimap<int, pair<int, int> >::iterator itr = pool.begin();
    linked[(itr->second).first] = 1;
    linked[(itr->second).second] = 1;
    //cout << "First two points added." << endl;
    mst.push_back(&p[(itr->second).first][(itr->second).second]);    //add the two points with the least distance between them
    pool.erase(itr);
    while (!pool.empty()) {
        for (itr = pool.begin(); itr != pool.end();) {
            if (linked[(itr->second).first] != linked[(itr->second).second])
                break;      //find the point that can connect to any points already adopted with the least distance
            if (linked[(itr->second).first] == 1) {
                multimap<int, pair<int, int> >::iterator temp = itr;
                ++itr;
                pool.erase(temp);  //if both ends of a path has already been adopted, exclude it from path pool
                //cout << "Worse path deleted." << endl;
            }
            else
                ++itr;
        }
        if (itr != pool.end()) {
            mst.push_back(&p[(itr->second).first][(itr->second).second]);
            linked[(itr->second).first] = linked[(itr->second).second] = 1;
            pool.erase(itr);    //adopt the point and corresponding path
            //cout << "New point added." << endl;
        }
    }
    //cout << "MST constructed." << endl;
    //opt(mst);
}

struct trace {
    int parent;
    int rank;
};

int findRoot(struct trace tracing[], int a) {
    if (tracing[a].parent != a)
        return findRoot(tracing, tracing[a].parent);
    return tracing[a].parent;
}

void kruskal(){
    //use Kruskal's algorithm to find MST
    multimap<int, pair<int, int> > pool;        //contains paths not excluded
    for (int i = 0; i < terms.size(); ++i)
        for (int j = i + 1; j < terms.size(); ++j){
            pool.insert(pair<int, pair<int, int> >(disRec[i][j], pair<int, int>(i, j)));
        }
    //cout << "Candidate pool constructed." << endl;
    int linked[terms.size()];                //keeps record of terminals already adopted
    for (int i = 0; i < terms.size(); ++i)
        linked[i] = 0;                      //initialization. None has been adopted

    struct trace *tracing = new struct trace[terms.size()];
    for (int i = 0; i < terms.size(); ++i) {
        tracing[i].parent = i;
        tracing[i].rank = 0;
    }

    multimap<int, pair<int, int> >::iterator itr;
    for (int i = 0; i < terms.size() - 1;) {    //will add N-1 paths to the MST
        for (itr = pool.begin(); itr != pool.end();) {
            if (findRoot(tracing, (itr->second).first) == findRoot(tracing, (itr->second).second)) {//if two ends of a path shares the same root, there will be a cycle formed after adding this path to the MST
                ++itr;      //just skip this path
                //cout << "Worse path skiped." << endl;
            }
            else {          //add the path to the MST
                int roota = findRoot(tracing, (itr->second).first);
                int rootb = findRoot(tracing, (itr->second).second);
                if (roota < rootb)
                    tracing[roota].parent = rootb;
                else if (roota > rootb)
                    tracing[rootb].parent = roota;
                else {
                    tracing[rootb].parent = roota;
                    tracing[roota].rank++;
                }           //to avoid too much imbalance in the tracing tree
                mst_k.push_back(&p[(itr->second).first][(itr->second).second]);
                ++i;        //new path added to the MST
                ++itr;      //go to next path
                //cout << "New path added." << endl;
            }
        }
    }
    //cout << "MST constructed." << endl;
}

//########

void draw() {
/* initialize display */
 init_graphics("Multiple terminals routing");


 init_world (0.,0.,1000.,1000.);
 //update_message("Interactive graphics example number 2!.");
 //create_button ("Window", "Next", new_button_func);
 drawscreen(); 
 event_loop(button_press, drawscreen);    
    
}

static void drawscreen (void) {

/* redrawing routine for still pictures.  Graphics package calls  *
 * this routine to do redrawing after the user changes the window *
 * in any way.                                                    */

 clearscreen();  /* Should be first line of all drawscreens */
 setfontsize (10);
 setlinestyle (SOLID);
 setlinewidth (0);
 setcolor (BLACK);
 int windowsize=WINDOWSIZE;
 int gridsize=(windowsize/upperright->x > windowsize/upperright->y)?
        (windowsize/upperright->y*.8):(windowsize/upperright->x*.8);

 //grid
 for (int i = 0; i < upperright->x; i++) {
    drawline(gridsize+i*gridsize, windowsize-gridsize, gridsize+i*gridsize, windowsize-upperright->y*gridsize);
 }

 for (int i = 0; i < upperright->y; i++) {
    drawline(gridsize, windowsize-gridsize-i*gridsize, upperright->x*gridsize, windowsize-gridsize-i*gridsize);
 }

 //obstacles
 for (ObstacleVector::iterator oi = obs.begin(); oi != obs.end(); oi++) {
    setcolor(RED);
    fillrect((*oi)->ll->x*gridsize, windowsize-(*oi)->ll->y*gridsize, (*oi)->ur->x*gridsize+gridsize, windowsize-(*oi)->ur->y*gridsize-gridsize);
 }   
 
 //MST
 for (PathVector::iterator spi = mst.begin(); spi != mst.end(); spi++) {
    Path* path = *spi;
    for (Path::iterator pi = path->begin(); pi != path->end(); pi++) {
        setcolor(BLUE);
        fillrect((*pi)->x*gridsize, windowsize-(*pi)->y*gridsize, (*pi)->x*gridsize+gridsize, windowsize-(*pi)->y*gridsize-gridsize);
        flushinput();
    }
 }

 //terminals
 for (Path::iterator pi = terms.begin(); pi != terms.end(); pi++) {
    setcolor(YELLOW);
    fillrect((*pi)->x*gridsize, windowsize-(*pi)->y*gridsize, (*pi)->x*gridsize+gridsize, windowsize-(*pi)->y*gridsize-gridsize);
    flushinput();
    //delay();
 } 

}

static void delay (void) {

/* A simple delay routine for animation. */

 int i, j, k, sum;

 sum = 0;
 for (i=0;i<50000;i++) 
    for (j=0;j<i;j++)
       for (k=0;k<30;k++) 
          sum = sum + i+j-k; 
}

static void button_press (float x, float y) {

/* Called whenever event_loop gets a button press in the graphics *
 * area.  Allows the user to do whatever he/she wants with button *
 * clicks.                                                        */
 int windowsize=WINDOWSIZE;
 int gridsize=(windowsize/upperright->x > windowsize/upperright->y)?
        (windowsize/upperright->y*.8):(windowsize/upperright->x*.8);
 int ax = int(x) / gridsize;
 int ay = (windowsize - int(y)) / gridsize;

 printf("User clicked a button at grid (%d,%d)\n", ax, ay);
}


static void new_button_func (void (*drawscreen_ptr) (void)) {

 printf ("You pressed the new button!\n");
 setcolor (MAGENTA);
 setfontsize (12);
 drawtext (500., 500., "You pressed the new button!", 10000.);
}

double mazegetTime() {
    struct rusage r;
    getrusage(RUSAGE_SELF, &r);
    
    return static_cast<double>(r.ru_utime.tv_sec)*1000000 + static_cast<double>(r.ru_utime.tv_usec);
}
