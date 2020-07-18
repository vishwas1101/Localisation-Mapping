# Mapping Algorithm 

This folder contains the implementations of mapping algorithms. 

## Quad-Tree Implementation

This is an algorithm used for environment mapping. Once the obstacle positions are specified, the entire environment is divide into 4 parts and each part is checked if any part of any obstacle is present. If yes, the the part is further divided into 4 parts and this process is repeated until the smallest dimensions of the part is obtained. Once this operation is compelete, we obtain a rough map of the environment as the part with the obstacles are seperated from the parts that are free. Like pixels in an image. The environment is now mapped.

The current QuadTree_implementation.m program creates a random obtacle map and can be modified to use a pre-defined environment map. 

