# CSE306 Raytracer Project
 Coding a simple raytracer in C++

Scene and lights modifiable in main() function, as well as movement functions for objects (motion blur). Accepts no command line arguments (for now). ".obj" meshes loadable using TriangleMesh class.

TD1: Shadows, mirrors done, transparency too

TD2: indirect lighting, antialiasing, depth of field, motion blur

TD3/4: cat rendered, rescaling function, bvhtree not visiting boxes "further" than best intersection found, uvs support, movement support for mesh

TODO: Check complexity with adding multiple lights/randomly pick lights | compute refraction |stuff within Sphere? | ++i instead of i++ | remove intersection.flag (t is set to Max anyway)?
TODO: add arguments passing through call instead of recompiling
TODO: make (SAH) bounding box for spheres too
TODO: procedural textures, the BRDFs they talk about, the importance sampling stuff
TODO: fix camera in a lens / lens clipping on wall?
TODO: add bokeh shapes
TODO: rotation of objects? (maybe the naive way)
