// ---------------------------------------------------------------------------
// QuadTree.h
// A QuadTree implementation.
//
// Based on examples from Wikipedia (http://en.wikipedia.org/wiki/Quadtree)
//
// This file is available on Github: https://github.com/antsam/universe
//
// Author: Anton Samson <anton@antonsamson.com>
// Created: January 17, 2013
// ---------------------------------------------------------------------------
#ifndef QUADTREE_H
#define QUADTREE_H

#include <cmath>
#include <vector>

#include "universe.h"

namespace Anton
{
    // essentially a pair or a tuple for x,y
    struct coord
    {
        double x, y;
        coord() : x(0), y(0) {}
        coord(const double &_x, const double &_y) : x(_x), y(_y) {}
    };

    // bounding box that encompasses a QuadTree
    struct box
    {
        coord centre;
        double width, height;
        // constructor with a coord
        box(const coord &_centre, const double &_width, const double &_height)
        {
            centre = _centre;
            if((_width > 0.0f) && (_height > 0.0f))
            {
                width = _width;
                height = _height;
            }
            else
            {
                width = 0;
                height = 0;
            }
        }
        // constructor with doubles for centre coordinates
        box(const double &x, const double &y, const double &_width, const double &_height)
        {
            centre.x = (x > 0) ? x : 0;
            centre.y = (y > 0) ? y : 0;

            if((_width > 0.0f) && (_height > 0.0f))
            {
                width = _width;
                height = _height;
            }
            else
            {
                width = 0;
                height = 0;
            }
        }
        box() : width(0), height(0) {}
        bool dimensions_set() const { return (width > 0.0f) && (height > 0.0f); }
        double min_x() const { return dimensions_set() ? (centre.x - (width/2.0f)) : 0; }
        double max_x() const { return dimensions_set() ? (centre.x + (width/2.0f)) : 0; }
        double min_y() const { return dimensions_set() ? (centre.y - (height/2.0f)) : 0; }
        double max_y() const { return dimensions_set() ? (centre.y + (height/2.0f)) : 0; }
        bool in_range(const double &value, const double &min, const double &max) const
        {
            return ((value >= min) && (value <= max));
        }
        bool inside_range(const double &value, const double &min, const double &max) const
        {
            return ((value > min) && (value < max));
        }
        // does the current bounding box contain an x-y coordinate?
        bool contains_coord(const coord &p) const
        {
            if(dimensions_set())
            {
                return (inside_range(p.y, min_y(), max_y()) && inside_range(p.x, min_x(), max_x()));
            }

            return false;
        }
        bool contains_coord(const double &x, const double &y) const
        {
            coord p(x, y);

            if(dimensions_set())
            {
                return (inside_range(y, min_y(), max_y()) && inside_range(x, min_x(), max_x()));
            }

            return false;
        }
        // checks if a bounding box intersects on any of the 4 sides of another
        bool intersects(const box &other) const
        {
            if(other.dimensions_set() && dimensions_set())
            {
                bool overlap_x = (in_range(min_x(), other.min_x(), other.max_x())
                                  || (in_range(other.min_x(), min_x(), max_x())));

                bool overlap_y = (in_range(min_y(), other.min_y(), other.max_y())
                                  || (in_range(other.min_y(), min_y(), max_y())));

                return (overlap_x && overlap_y);
            }

            return false;
        }
    };

    class QuadTree
    {
        public:
            QuadTree(const box &bounds, const std::size_t &max_leaves);
            virtual ~QuadTree();
            bool add_leaf(Uni::Robot *r);
            std::vector<Uni::Robot *> get_leaves_at(const coord &p);
            std::vector<Uni::Robot *> get_leaves_at(const double &x, const double &y);
            std::vector<Uni::Robot *> get_leaves_at(const box &b);
            std::vector<Uni::Robot *> find_in_range(const coord &p);
            std::vector<Uni::Robot *> find_in_range(const double &x, const double &y);
            std::vector<Uni::Robot *> find_in_range(const box &b);
            void clear();
            void flush();
            size_t get_max_leaves() const;
        protected:
        private:
            QuadTree();
            QuadTree(const QuadTree &other);
            QuadTree operator=(const QuadTree &other);
            std::vector<Uni::Robot *> leaves;
            box bounds;
            size_t max_leaves; // the max number of elements in leaves before we subdivide the tree
            QuadTree *northeast, *northwest, *southeast, *southwest;
            void subdivide();
    };
}

#endif // QUADTREE_H
