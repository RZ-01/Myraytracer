# Myraytracer
## Introduction
Create a raytracer with bare C++ (No Graphics Library Included), built to handle various materials, light sources, and geometric shapes, providing a foundational framework for generating realistic images.
## Features
- **Realistic Material Simulation**: The ray tracer supports various materials, each with unique optical properties, includes:
  - Diffuse surfaces that scatter light in many directions.
  - Reflective materials that create mirrors and shiny surfaces.
  - Transparent materials with refraction to simulate glass and liquids.

- **Dynamic Lighting**: Implements point light sources with adjustable intensity and position.

- **Environment Mapping**: Utilizes high-resolution environment maps to simulate distant scenery and reflections, adding depth and immersion to the scenes.

- **Reflection and Refraction**: Accurately models the reflection and refraction of light, taking into account the material's specular characteristics and index of refraction for realistic visual effects.

## Results
![4k](./out_4k.png)

## Usage
- Download all files and open .sln project (visual studio)
- Or Download Geo.h and main.cpp (only 2 files needed) and compile on your own 

## Future Work

- **Anti-Aliasing**: To smooth out jagged edges and improve image quality.
- **More Geometric Primitives**: Adding support for planes, boxes, and triangles to create more complex scenes.
- **Advanced Material Models**: Implementation of more sophisticated material models including subsurface scattering and metallic reflections.
- **High-Performance Parallel Computing**: Utilize the computational power of modern GPUs (CUDA), enabling real-time rendering capabilities even for complex scenes with high levels of detail.

