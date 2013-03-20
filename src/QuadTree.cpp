// ---------------------------------------------------------------------------
// QuadTree.cpp
// A QuadTree implementation.
//
// Based on examples from Wikipedia (http://en.wikipedia.org/wiki/Quadtree)
//
// This file is available on Github: https://github.com/antsam/universe
//
// Author: Anton Samson <anton@antonsamson.com>
// Created: January 17, 2013
// ---------------------------------------------------------------------------
#include "QuadTree.h"
#include <algorithm>

using namespace Anton;

const std::size_t DEFAULT_MAX_LEAVES = 10;

QuadTree::QuadTree(const box &bounds, const std::size_t &max_leaves)
{
    if(bounds.dimensions_set())
    {
        this->bounds = bounds;
    }

    this->max_leaves = ((max_leaves > 0) && (max_leaves > DEFAULT_MAX_LEAVES)) ? max_leaves : DEFAULT_MAX_LEAVES;

    this->northwest = NULL;
    this->northeast = NULL;
    this->southwest = NULL;
    this->southeast = NULL;
}

// delete the trees on destruction
QuadTree::~QuadTree()
{
    if(!this->leaves.empty())
    {
        this->leaves.clear();
    }

    this->clear();
}

bool QuadTree::add_leaf(Uni::Robot *r)
{
    if(!this->bounds.contains_coord(r->pose[0], r->pose[1]))
    {
        return false;
    }

    if(this->leaves.size() < this->max_leaves)
    {
        this->leaves.push_back(r);
        return true;
    }

    if(this->northwest == NULL)
    {
        this->subdivide();
    }

    if(this->northwest->add_leaf(r))
    {
        return true;
    }

    if(this->northeast->add_leaf(r))
    {
        return true;
    }

    if(this->southwest->add_leaf(r))
    {
        return true;
    }

    if(this->southeast->add_leaf(r))
    {
        return true;
    }

    // if we got here, something really bad happened

    return false;
}

std::vector<Uni::Robot *> QuadTree::get_leaves_at(const box &b)
{
    std::vector<Uni::Robot *> found;

    if(!this->bounds.intersects(b))
    {
        return found;   // not in this quadrant
    }

    std::size_t i = 0, local_leaves = this->leaves.size();
    for(; i < local_leaves; ++i)
    {
        if(b.contains_coord(this->leaves[i]->pose[0], this->leaves[i]->pose[1]))
        {
            found.push_back(this->leaves[i]);
        }
    }

    if(this->northwest == NULL)
    {
        return found;
    }

    std::vector<Uni::Robot *> nw = this->northwest->get_leaves_at(b);
    std::vector<Uni::Robot *> ne = this->northeast->get_leaves_at(b);
    std::vector<Uni::Robot *> sw = this->southwest->get_leaves_at(b);
    std::vector<Uni::Robot *> se = this->southeast->get_leaves_at(b);

    found.insert(found.end(), nw.begin(), nw.end());
    found.insert(found.end(), ne.begin(), ne.end());
    found.insert(found.end(), sw.begin(), sw.end());
    found.insert(found.end(), se.begin(), se.end());

    return found;
}

// polymorphic if you want it.
std::vector<Uni::Robot *> QuadTree::get_leaves_at(const coord &p)
{
    double search_range = 2 * Uni::Robot::range;
    box range(p, search_range, search_range);

    return this->get_leaves_at(range);
}

std::vector<Uni::Robot *> QuadTree::get_leaves_at(const double &x, const double &y)
{
    double search_range = 2 * Uni::Robot::range;
    box range(x, y, search_range, search_range);

    return this->get_leaves_at(range);
}

std::vector<Uni::Robot *> QuadTree::find_in_range(const coord &p)
{
    double search_range = 2 * Uni::Robot::range;
    box range(p, search_range, search_range);

    return this->find_in_range(range);
}

std::vector<Uni::Robot *> QuadTree::find_in_range(const double &x, const double &y)
{
    double search_range = 2 * Uni::Robot::range;
    box range(x, y, search_range, search_range);

    return this->find_in_range(range);
}

// Find any robots that may be in the torus range
std::vector<Uni::Robot *> QuadTree::find_in_range(const box &b)
{
    std::vector<Uni::Robot *> found;

    found = this->get_leaves_at(b);

    if((b.min_x() >= 0.0f) && (b.min_y() >= 0.0f) && (b.max_x() <= 1.0f) && (b.max_y() <= 1.0f))
    {
        return found;
    }

    // deal with robots in torus
    std::vector<Uni::Robot *> overflow;
    box query;

    if(b.min_x() < 0.0f)
    {
        query.width = b.min_x() * (-2);
        query.height = b.height;
        query.centre.x = 1;
        query.centre.y = b.centre.y;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }
    else if(b.max_x() > 1.0f)
    {
        query.width = (1 - b.min_x()) * 2;
        query.height = b.height;
        query.centre.x = 0;
        query.centre.y = b.centre.y;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }

    if(b.min_y() < 0.0f)
    {
        query.width = b.width;
        query.height = b.min_y() * (-2);
        query.centre.x = b.centre.x;
        query.centre.y = 1;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }
    else if(b.max_y() > 1.0f)
    {
        query.width = b.width;
        query.height = (1 - b.min_y()) * 2;
        query.centre.x = b.centre.x;
        query.centre.y = 0;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }

    if((b.min_x() < 0.0f) && (b.min_y() < 0.0f))
    {
        query.width = b.min_x() * (-2);
        query.height = b.min_y() * (-2);
        query.centre.x = 1;
        query.centre.y = 1;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }
    else if((b.max_x() > 1.0f) && (b.max_y() > 1.0f))
    {
        query.width = (1 - b.min_x()) * 2;
        query.height = (1 - b.min_y()) * 2;
        query.centre.x = 0;
        query.centre.y = 0;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }
    else if((b.min_x() < 0.0f) && (b.max_y() > 1.0f))
    {
        query.width = b.min_x() * (-2);
        query.height = (1 - b.min_y()) * 2;
        query.centre.x = 1;
        query.centre.y = 0;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }
    else if((b.min_y() < 0.0f) && (b.max_x() > 1.0f))
    {
        query.width = (1 - b.min_x()) * 2;
        query.height = b.min_y() * (-2);
        query.centre.x = 0;
        query.centre.y = 1;
        overflow = this->get_leaves_at(query);

        if(!overflow.empty())
        {
            found.insert(found.end(), overflow.begin(), overflow.end());
            overflow.clear();
        }
    }

    return found;
}

// clear all the leaf vectors and then delete the trees
void QuadTree::clear()
{
    if(!this->leaves.empty())
    {
        this->leaves.clear();
        std::vector<Uni::Robot *> (this->leaves).swap(this->leaves);
    }

    if(this->northwest != NULL)
    {
        this->northwest->clear();
        delete this->northwest;
        this->northwest = NULL;
    }

    if(this->northeast != NULL)
    {
        this->northeast->clear();
        delete this->northeast;
        this->northeast = NULL;
    }

    if(this->southwest != NULL)
    {
        this->southwest->clear();
        delete this->southwest;
        this->southwest = NULL;
    }

    if(this->southeast != NULL)
    {
        this->southeast->clear();
        delete this->southeast;
        this->southeast = NULL;
    }
}

// clear all the leaf vectors but leave the trees intact
void QuadTree::flush()
{
    if(!this->leaves.empty())
    {
        this->leaves.clear();
        std::vector<Uni::Robot *> (this->leaves).swap(this->leaves);
    }

    if(this->northwest != NULL)
    {
        this->northwest->clear();
    }

    if(this->northeast != NULL)
    {
        this->northeast->clear();
    }

    if(this->southwest != NULL)
    {
        this->southwest->clear();
    }

    if(this->southeast != NULL)
    {
        this->southeast->clear();
    }
}

size_t QuadTree::get_max_leaves() const
{
    return this->max_leaves;
}

// PRIVATE FUNCTIONS

// divide the current bounding box into 4 equal boxes.
void QuadTree::subdivide()
{
    // need to simplify this
    double new_width = this->bounds.width/2.0f;
    double new_height = this->bounds.height/2.0f;
    double half_width = new_width/2.0f;
    double half_height = new_height/2.0f;
    double x = this->bounds.centre.x;
    double y = this->bounds.centre.y;

    box nw((x - half_width), (y + half_height), new_width, new_height);
    box ne((x + half_width), (y + half_height), new_width, new_height);
    box sw((x - half_width), (y - half_height), new_width, new_height);
    box se((x + half_width), (y - half_height), new_width, new_height);

    this->northwest = new QuadTree(nw, max_leaves);
    this->northeast = new QuadTree(ne, max_leaves);
    this->southwest = new QuadTree(sw, max_leaves);
    this->southeast = new QuadTree(se, max_leaves);
}
