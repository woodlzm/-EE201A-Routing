#define MAX 0xffff
#define WINDOWSIZE 1000
#include <vector>
#include <set>

using namespace std;

class Point;
class Obstacle;

typedef vector<Point*>          Path;
typedef set<Point*>             PointSet;
typedef vector<Obstacle*>       ObstacleVector;
typedef vector<Path*>           PathVector;

struct LessPath {
    bool operator ()(const Path* left, const Path* right) const {
        return left->size() < right->size();
    }
};

typedef set<Path*, LessPath>    PathSet;

class Point {
    public:
        Point(int a, int b):
            x(a),
            y(b) {}

        Point(Point* p):
            x(p->x),
            y(p->y) {}
        
        Point()
            {
                Point(0,0);
            }

    public:
        int x;
        int y;
        PathVector* paths;
};

class Obstacle {
    public:
        Obstacle(int llx, int lly, int urx, int ury)
            {
                ll = new Point(llx, lly);
                ur = new Point(urx, ury);
            }

        Obstacle(Point* a, Point* b):
            ll(a),
            ur(b) {
            if(a->x > b->x) {
                ll = new Point(b);
                ur = new Point(a);
            }
            else {
                ll = new Point(a);
                ur = new Point(b);
            } 
        }

        ~Obstacle() {
            if (ll) delete ll;
            if (ur) delete ur;
        }

    public:
        Point* ll;      //the lower left corner
        Point* ur;      //the upper right corner
};

void init(char *fn);

Path* maze(Point* s, Point* t);

Path* retrace(Point* s, Point* t);

void print_graph();

void print_path(Path* path);

void clean();

void buildGraph();

void prim();

void kruskal();

void draw();

static void delay (void);

static void button_press (float x, float y);

static void drawscreen (void);

static void new_button_func (void (*drawscreen_ptr) (void)); 

double mazegetTime();
