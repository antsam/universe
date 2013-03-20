Universe
========

A simple n-body simulator for exploring distributed systems.

Original Author: Richard Vaughan 2013.
License: GNU GPL v3 or later (applies to all files in this repo).

Modified to use QuadTrees by Anton Samson. The next iteration makes
use of Spatial Hashing and multi-threading and may be pushed
after the semester ends.

## Pre-requirements

### Ubuntu (Tested on Ubuntu 11.10, 12.04 and 12.10)

<pre>
sudo apt-get install build-essential cmake freeglut3-dev 
</pre>

Optional: 

<pre>
sudo apt-get install libxi-dev libxmu-dev
</pre>

### Mac OS (Tested on 10.8)

Needs XCode command-line tools.

## Compile

```bash
[in code directory]
make
```

The Make script is a wrapper around [CMake](http://www.cmake.org/). 
