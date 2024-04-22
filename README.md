# CSE306 Raytracer Project
 Coding a simple raytracer in C++

".obj" meshes loadable using TriangleMesh class. Run with argument "help" for help on using the program.
Scene is modifiable in "main" function.

TD1: Shadows, mirrors done, refraction and fresnel

TD2: indirect lighting, antialiasing

TD3/4: cat rendered, rescaling function, bvhtree not visiting boxes "further" than best intersection found, uvs support, movement support for mesh

depth of field, motion blur, SAH bounding box algorithm (20s -> 16s on example scene)

TODO: make (SAH) bounding box for spheres too
TODO: procedural textures, the BRDFs they talk about, the importance sampling stuff
TODO: fix camera in a lens / lens clipping on wall?
TODO: add bokeh shapes
TODO: rotation of objects? (maybe the naive way)
