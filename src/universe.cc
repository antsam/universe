/****
         universe.cc
         version 3
         Richard Vaughan

         modified by Anton Samson (https://github.com/antsam/universe)
****/

#include <cassert>
#include <unistd.h>
#include <iostream>
#include <sys/time.h>
#include "universe.h"
#include "QuadTree.h"

const int period = 10;  // for timing FPS

Anton::QuadTree *tree;
std::vector<Uni::Robot *> quadrant, quadrant_overflow;

using namespace Uni;

const char* PROGNAME = "universe";

#if GRAPHICS
    #ifdef __APPLE__
        #include <glut/glut.h>
    #else
        #include <GL/glut.h> // OS X users need <glut/glut.h> instead
    #endif
#endif

namespace Uni
{
    bool need_redraw(true);
    double worldsize(1.0);
    std::vector<Robot> population(100); // why are we defaulting to 100?
    uint64_t updates(0);
    uint64_t updates_max(0.0);
    bool paused(false);
    int winsize(600);
    int displaylist(0);
    bool show_data(true);
    unsigned int sleep_msec(50);
    double lastseconds;

    // Robot static members
    unsigned int Robot::pixel_count(8);
    double Robot::range(0.1);
    double Robot::fov(dtor(270.0));
}

char usage[] = "Universe understands these command line arguments:\n"
    "    -? : Prints this helpful message.\n"
    "    -c <int> : sets the number of pixels in the robots' sensor.\n"
    "    -d    Disables drawing the sensor field of view. Speeds things up a bit.\n"
    "    -f <float> : sets the sensor field of view angle in degrees.\n"
    "    -p <int> : set the size of the robot population.\n"
    "    -q : disables chatty status output (quiet mode).\n"
    "    -r <float> : sets the sensor field of view range.\n"
    "    -s <float> : sets the side length of the (square) world.\n"
    "    -u <int> : sets the number of updates to run before quitting.\n"
    "    -w <int> : sets the initial size of the window, in pixels.\n"
    "    -z <int> : sets the number of milliseconds to sleep between updates.\n";

#if GRAPHICS
// GLUT callback functions ---------------------------------------------------

// update the world - this is called whenever GLUT runs out of events
// to process
static void idle_func()
{
    Uni::UpdateAll();
    // possibly snooze to save CPU and slow things down
    if(Uni::sleep_msec > 0)
        usleep(Uni::sleep_msec * 1e3);
}

static void timer_func(int dummy)
{
    glutPostRedisplay(); // force redraw
}

// draw the world - this is called whenever the window needs redrawn
static void display_func()
{
    if(Uni::need_redraw)
    {
        Uni::need_redraw = false;

        glClear(GL_COLOR_BUFFER_BIT);

        FOR_EACH(r, population)
        {
            r->Draw();
        }

        glutSwapBuffers();

        glFlush();
    }

    // cause this run again in about 50 msec
    glutTimerFunc(50, timer_func, 0);
}

static void mouse_func(int button, int state, int x, int y)
{
    if((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN))
    {
        Uni::paused = !Uni::paused;
    }
}

#endif // GRAPHICS

Robot::Robot()
    : pose(),
        speed(),
        color(),
        pixels(pixel_count),
        callback(NULL),
        callback_data(NULL)
{
    // until C++ supports array literals in the initialization list, we're forced to do this
    memset(pose, 0, sizeof(pose));
    memset(speed, 0, sizeof(speed));
    color[0] = 128;
    color[1] = 0;
    color[2] = 0;
}

void Uni::Init( int argc, char** argv )
{
    // seed the random number generator with the current time
    //srand48(time(NULL));
    srand48(0);

    bool quiet = false; // controls output verbosity

    int population_size = 100;
    population.resize(population_size);

    // parse arguments to configure Robot static members
    // opterr = 0; // supress errors about bad options
    int c;
    while((c = getopt(argc, argv, ":?dqp:s:f:r:c:u:z:w:")) != -1)
    {
        switch( c )
        {
            case 'p':
                population_size = atoi( optarg );
                if(!quiet) printf( "[Uni] population_size: %d\n", population_size );
                population.resize( population_size );
                break;
            case 's':
                worldsize = atof( optarg );
                if(!quiet) printf( "[Uni] worldsize: %.2f\n", worldsize );
                break;
            case 'f':
                Robot::fov = dtor(atof( optarg )); // degrees to radians
                if(!quiet) printf( "[Uni] fov: %.2f\n", Robot::fov );
                break;
            case 'r':
                Robot::range = atof( optarg );
                if(!quiet) printf( "[Uni] range: %.2f\n", Robot::range );
                break;
            case 'c':
                Robot::pixel_count = atoi( optarg );
                if(!quiet) printf( "[Uni] pixel_count: %d\n", Robot::pixel_count );
                break;
            case 'u':
                updates_max = atol( optarg );
                if(!quiet) printf( "[Uni] updates_max: %lu\n", (long unsigned)updates_max );
                break;
            case 'z':
                sleep_msec = atoi( optarg );
                if(!quiet) printf( "[Uni] sleep_msec: %d\n", sleep_msec );
                break;
#if GRAPHICS
            case 'w':
                winsize = atoi( optarg );
                if(!quiet) printf( "[Uni] winsize: %d\n", winsize );
                break;
            case 'd':
                show_data= false;
                if(!quiet) puts( "[Uni] hide data" );
                break;
            case 'q': quiet = true;
                break;
#endif
            case '?':
                puts( usage );
                exit(0); // ok
                break;
            default:
                fprintf( stderr, "[Uni] Option parse error.\n" );
                puts( usage );
                exit(-1); // error
        }
    }

#if GRAPHICS
    // initialize opengl graphics
    glutInit(&argc, argv);
    glutInitWindowSize(winsize, winsize);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
    glutCreateWindow(PROGNAME);
    glClearColor(0.8, 0.8, 1.0, 1.0);
    glutDisplayFunc(display_func);
    glutTimerFunc(50, timer_func, 0);
    glutMouseFunc(mouse_func);
    glutIdleFunc(idle_func);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, 1, 0, 1);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glScalef(1.0/worldsize, 1.0/worldsize, 1);

    // define a display list for a robot body
    double h = 0.01;
    double w = 0.01;

    glPointSize( 4.0 );

    displaylist = glGenLists(1);
    glNewList(displaylist, GL_COMPILE);

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    glBegin(GL_POLYGON);
    glVertex2f(h/2.0, 0);
    glVertex2f(-h/2.0, w/2.0);
    glVertex2f(-h/2.0, -w/2.0);
    glEnd();

    glEndList();
#endif // GRAPHICS

    struct timeval start;
    gettimeofday(&start, NULL);
    lastseconds = start.tv_sec + start.tv_usec/1e6;
}

void Robot::UpdateSensor()
{
    quadrant.clear();

    double radians_per_pixel = fov / (double)pixel_count;
    double halfworld = worldsize * 0.5f;

    // initialize pixels vector
    FOR_EACH(it, pixels)
    {
        it->range = Robot::range; // maximum range
        it->robot = NULL; // nothing detected
    }

    double search_range = (Robot::range * 2);

    Anton::box query(pose[0], pose[1], search_range, search_range);
    //quadrant = tree->get_leaves_at(query);
    quadrant = tree->find_in_range(query);  // find any robots in torus range

    std::size_t i = 0, quadrant_size = quadrant.size();
    double dx, dy, range, absolute_heading, relative_heading;
    int pixel;

    // check every robot near by to see if it is detected
    for(; i < quadrant_size; ++i)
    {
        Robot *other = quadrant[i];

        // discard if it's the same robot
        if(other == this)
        {
            continue;
        }

        // discard if it's out of range. We put off computing the
        // hypotenuse as long as we can, as it's relatively expensive

        dx = other->pose[0] - pose[0];

        // wrap around torus
        if(dx > halfworld)
            dx -= worldsize;
        else if(dx < -halfworld)
            dx += worldsize;

        if(fabs(dx) > Robot::range)
        {
            continue;   // out of range
        }

        dy = other->pose[1] - pose[1];

        // wrap around torus
        if(dy > halfworld)
        {
            dy -= worldsize;
        }
        else if(dy < -halfworld)
        {
            dy += worldsize;
        }

        if(fabs(dy) > Robot::range)
        {
            continue;   // out of range
        }

        range = hypot(dx, dy);

        if(range > Robot::range)
        {
            continue;
        }

        // discard if it's out of field of view
        absolute_heading = atan2(dy, dx);
        relative_heading = AngleNormalize((absolute_heading - pose[2]));

        if(fabs(relative_heading) > (fov/2.0))
        {
            continue;
        }

        // find which pixel it falls in
        pixel = floor( relative_heading / radians_per_pixel );
        pixel += pixel_count / 2;
        pixel %= pixel_count;

        assert(pixel >= 0);
        assert(pixel < (int)pixel_count);

        // discard if we've seen something closer in this pixel already.
        if(pixels[pixel].range < range)
        {
            continue;
        }

        // if we made it here, we see this other robot in this pixel.
        pixels[pixel].range = range;
        pixels[pixel].robot = other;
    }
}

void Robot::UpdatePose()
{
    // move according to the current speed

    pose[0] = DistanceNormalize(pose[0] + (speed[0] * cos(pose[2])));   // pose[0] + dx
    pose[1] = DistanceNormalize(pose[1] + (speed[0] * sin(pose[2])));   // pose[1] + dy
    pose[2] = AngleNormalize(pose[2] + speed[1]);   // pose[2] + da
}

void Uni::UpdateAll()
{
    // if we've done enough updates, exit the program
    if((updates_max > 0) && (updates > updates_max))
    {
        /*FOR_EACH(r, population)
        {
            std::cout << r->pose[0] << " " << r->pose[1] << std::endl;
        }*/
        exit(1);
    }

    if(!paused)
    {
        FOR_EACH(r, population)
        {
            r->UpdatePose();
        }

        // add robots to tree
        FOR_EACH(it, population)
        {
            tree->add_leaf(&(*it));
        }

        FOR_EACH(r, population)
        {
            r->UpdateSensor();
        }

        //tree->clear();
        tree->flush();

        FOR_EACH(r, population)
        {
            Robot &b = *r;
            r->callback(b, r->callback_data);
        }

        need_redraw = true;

        if((updates % period) == 0)
        {
            struct timeval now;
            gettimeofday( &now, NULL );
            double seconds = now.tv_sec + now.tv_usec/1e6;
            double interval = seconds - lastseconds;
            printf("[%d] FPS %.3f\r", (int)updates, (period/interval));
            fflush(stdout);
            lastseconds = seconds;
        }
    }

    ++updates;
}

// draw a robot
void Robot::Draw() const
{
#if GRAPHICS
    glPushMatrix();
    glTranslatef(pose[0], pose[1], 0);
    glRotatef(rtod(pose[2]), 0, 0, 1);

    glColor3ub(color[0], color[1], color[2]);

    // draw the pre-compiled triangle for a body
    glCallList(displaylist);

    if(show_data)
    {
        // render the sensors
        double rads_per_pixel = fov / (double)pixel_count;
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        double angle, dx1, dy1, dx2, dy2;
        double half_rads_per_pixel = (rads_per_pixel/2.0);
        unsigned int p = 0;

        for(; p < pixel_count; ++p)
        {
            angle = -fov/2.0 + (p+0.5) * rads_per_pixel;
            dx1 = pixels[p].range * cos(angle + half_rads_per_pixel);
            dy1 = pixels[p].range * sin(angle + half_rads_per_pixel);
            dx2 = pixels[p].range * cos(angle - half_rads_per_pixel);
            dy2 = pixels[p].range * sin(angle - half_rads_per_pixel);

            glColor4f( 1,0,0, pixels[p].robot ? 0.2 : 0.05 );

            glBegin(GL_POLYGON);
            glVertex2f(0, 0);
            glVertex2f(dx1, dy1);
            glVertex2f(dx2, dy2);
            glEnd();
        }
    }

    glPopMatrix();
#endif // GRAPHICS
}

void Uni::Run()
{
    double half_dimension = 1.0/2.0f;
    unsigned int max_leaves = 10;
    Anton::box grid(half_dimension, half_dimension, 1.0, 1.0);
    tree = new Anton::QuadTree(grid, max_leaves);
    //std::cout << "Population: " << population.size() << std::endl;
    //std::cout << "Max leaves: " << tree->get_max_leaves() << std::endl;
#if GRAPHICS
    glutMainLoop();
#else
    while(1)
    {
        Uni::UpdateAll();
    }
#endif
}
