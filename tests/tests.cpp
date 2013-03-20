// ---------------------------------------------------------------------------
// tests.cpp
// Testing the basic functions of QuadTree
//
// This file is available on Github: https://github.com/antsam/universe
//
// Author: Anton Samson <anton@antonsamson.com>
// Created: January 17, 2013
// ---------------------------------------------------------------------------
#include "src/QuadTree.h"
#include <cassert>
#include <iostream>

#define VAR(V,init) __typeof(init) V=(init)
#define FOR_EACH(I,C) for(VAR(I,(C).begin());I!=(C).end();I++)

/**
 * Generally I would use a testing framework for this but I don't know if we can install
 * libraries in CSIL properly.
 */

int main()
{
    /**
     * Should be the following:
     * (1,3),(2,3),(3,3)
     * (1,2),(2,2),(3,2)
     * (1,1),(2,1),(3,1)
     */
    std::cout << "Setting up the following grid:" << std::endl;
    std::cout << " (1,1),(2,1),(3,1)" << std::endl;
    std::cout << " (1,2),(2,2),(3,2)" << std::endl;
    std::cout << " (1,3),(2,3),(3,3)" << std::endl << std::endl;;
    Anton::box grid(2.0, 2.0, 3.0, 3.0);

    double x = 0.0f, y = 0.0f;

    // testing contains_coord()
    Anton::coord point;

    for(; x < 5; ++x)
    {
        for(; y < 5; ++y)
        {
            std::cout << "Testing if (" << x << "," << y << ")";
            point.x = x;
            point.y = y;
            if((x > 0) && (x < 4) && (y > 0) && (y < 4))
            {
                std::cout << " in grid.      ";
                assert(grid.contains_coord(point));
            }
            else
            {
                std::cout << " not in grid.  ";
                assert(!grid.contains_coord(point));
            }

            std::cout << "PASSED" << std::endl;
        }

        y = 0;
    }

    std::cout << std::endl;

    // testing intersects()
    std::size_t winsize = 10;
    Anton::box stationary(20,20, winsize, winsize);
    Anton::box moving(10,10, winsize, winsize);

    std::cout << "Box width and height set to " << winsize << std::endl;

    std::cout << "Stationary box setup at (" << stationary.centre.x << "," << stationary.centre.y << ")" << std::endl;
    std::cout << "Moving box setup at (" << moving.centre.x << "," << moving.centre.y << ")" << std::endl;

    // Side by side, borders touching
    std::cout << "Testing if borders are touching.                                ";
    assert(stationary.intersects(moving));
    assert(moving.intersects(stationary));
    std::cout << "PASSED" << std::endl;

    moving.centre.x = 9;
    std::cout << "Moving box setup at (" << moving.centre.x << "," << moving.centre.y << ")" << std::endl;
    std::cout << "Testing if borders are no longer touching.                      ";
    assert(!stationary.intersects(moving));
    assert(!moving.intersects(stationary));
    std::cout << "PASSED" << std::endl;

    moving.centre.x = 12;
    std::cout << "Moving box setup at (" << moving.centre.x << "," << moving.centre.y << ")" << std::endl;

    std::cout << "Testing if stationary intersects with right border of moving.   ";
    assert(stationary.intersects(moving));
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if moving intersects with left border of stationary.    ";
    assert(moving.intersects(stationary));
    std::cout << "PASSED" << std::endl;

    moving.centre.x = 32;
    std::cout << "Moving box setup at (" << moving.centre.x << "," << moving.centre.y << ")" << std::endl;

    std::cout << "Testing if stationary intersects with left border of moving.    ";
    assert(!stationary.intersects(moving));
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if moving intersects with right border of stationary.   ";
    assert(!moving.intersects(stationary));
    std::cout << "PASSED" << std::endl;

    moving.centre.x = 29;
    std::cout << "Moving box setup at (" << moving.centre.x << "," << moving.centre.y << ")" << std::endl;

    std::cout << "Testing if stationary intersects with left border of moving.    ";
    assert(stationary.intersects(moving));
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if moving intersects with right border of stationary.   ";
    assert(moving.intersects(stationary));
    std::cout << "PASSED" << std::endl;

    moving.centre.x = 10;
    moving.centre.y = 20;
    std::cout << "Moving box setup at (" << moving.centre.x << "," << moving.centre.y << ")" << std::endl;

    std::cout << "Testing if stationary intersects with bottom border of moving.  ";
    assert(stationary.intersects(moving));
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if moving intersects with top border of stationary.     ";
    assert(moving.intersects(stationary));
    std::cout << "PASSED" << std::endl;

    // Testing QuadTree
    std::size_t max_leaves = 8, leaves = max_leaves * 3;
    winsize = 1.0f;
    double half_winsize = winsize/2.0f;

    Anton::box canvas(half_winsize, half_winsize, winsize, winsize);
    Anton::QuadTree *tree = new Anton::QuadTree(canvas, max_leaves);

    std::vector<Uni::Robot> population(leaves);

    double normalized = 1.0f/6.0f;
    x = normalized;
    y = normalized;

    std::cout << std::endl << "Placing " << leaves << " robots in (" << x*600 << "," << y*600 << ")." << std::endl;

    FOR_EACH(it, population)
    {
        Uni::Robot *r = &(*it);
        r->pose[0] = x;
        r->pose[1] = y;

        tree->add_leaf(r);
    }

    std::vector<Uni::Robot *> found = tree->get_leaves_at(x, y);

    tree->clear();

    std::cout << "Testing if all robots found in (" << x*600 << "," << y*600 << ").            ";
    assert(found.size() == leaves);
    std::cout << "PASSED" << std::endl;

    population.resize(24);

    std::cout << std::endl << "Robot population flushed." << std::endl;
    std::cout << "Refer to grid.jpg in the main source folder to see new robot placements." << std::endl;

    std::vector<double> xs;
    std::vector<double> ys;

    xs.push_back(13.1871);
    ys.push_back(432.451);
    xs.push_back(472.319);
    ys.push_back(487.395);
    xs.push_back(12.5336);
    ys.push_back(250.92);
    xs.push_back(486.991);
    ys.push_back(205.098);
    xs.push_back(525.044);
    ys.push_back(552.935);
    xs.push_back(456.587);
    ys.push_back(229.741);
    xs.push_back(147.163);
    ys.push_back(155.651);
    xs.push_back(172.912);
    ys.push_back(434.628);
    xs.push_back(389.156);
    ys.push_back(278.204);
    xs.push_back(94.8658);
    ys.push_back(449.44);
    xs.push_back(390.668);
    ys.push_back(47.9782);
    xs.push_back(326.299);
    ys.push_back(135.428);
    xs.push_back(264.488);
    ys.push_back(475.037);
    xs.push_back(250.76);
    ys.push_back(316.419);
    xs.push_back(375.353);
    ys.push_back(41.4788);
    xs.push_back(69.8571);
    ys.push_back(106.895);
    xs.push_back(101.064);
    ys.push_back(337.218);
    xs.push_back(274.95);
    ys.push_back(317.633);
    xs.push_back(181.114);
    ys.push_back(577.256);
    xs.push_back(436.663);
    ys.push_back(284.437);
    xs.push_back(519.604);
    ys.push_back(285.999);
    xs.push_back(192.626);
    ys.push_back(351.266);
    xs.push_back(472.373);
    ys.push_back(351.097);
    xs.push_back(566.298);
    ys.push_back(16.7884);

    std::size_t i = 0;

    FOR_EACH(it, population)
    {
        Uni::Robot *r = &(*it);
        r->pose[0] = xs[i]/600.0f;
        r->pose[1] = ys[i]/600.0f;
        i++;

        tree->add_leaf(r);
    }

    std::cout << "Testing if 3 robots found in (" << xs[13] << "," << ys[13] << ").   ";
    found.clear();
    found = tree->get_leaves_at(xs[13]/600.0f, ys[13]/600.0f);
    assert(found.size() == 3);
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if 2 robots found in (" << xs[17] << "," << ys[17] << ").   ";
    found.clear();
    found = tree->get_leaves_at(xs[17]/600.0f, ys[17]/600.0f);
    assert(found.size() == 2);
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if 0 robots found in (0,0).              ";
    found.clear();
    found = tree->get_leaves_at(0, 0);
    assert(found.empty());
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if 1 robot near (0,0).                   ";
    found.clear();
    found = tree->find_in_range(0, 0);
    assert(found.size() == 1);
    std::cout << "PASSED" << std::endl;

    std::cout << "Testing if 0 robots found in (200,200).          ";
    found.clear();
    found = tree->get_leaves_at(200, 200);
    assert(found.empty());
    std::cout << "PASSED" << std::endl;

    tree->clear();

    delete tree;
    tree = NULL;

    xs.clear();
    ys.clear();

    std::cout << std::endl << "All tests passed!" << std::endl;

    return 0;
}
