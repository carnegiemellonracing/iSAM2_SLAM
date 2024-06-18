## What is iSAM2
iSAM2 (Incremental Smoothing and Mapping) is a SLAM (Simultaneous Localization and Mapping) algorithm used to construct a map of the track from the car's position and the cones observed around the track. iSAM2 does this by constructing a factor graph using variables nodes (which represent either landmark poses or car poses) and factor nodes.

### The Factor Graph
Factor nodes are connected to variables nodes and represents a probability distribution on the variables nodes connected to it. For example, factor node f_{1}_ is connected to variable node x_{1}_, representing the first car pose, and l_{1}_, representing the first landmark. f_{1}_ represents a joint probabilistic distribution function over x_{1}_ and l_{1}_). Altogether, the entire factor graph represents a joint probabilistic distribution function F on all variables (the landmarks poses and car poses) represented by the variable nodes. This function F is equal to the product of all factors f_{n}_, the joint probabilistic distribution function represented by each factor node in the graph.

## Goal
The goal of iSAM2 is to maximize the joint probabilistic distribution function F by maximizing its factors. Intuitively, iSAM2 is seeking to maximize its certainty of landmark positions and car poses by maximizing these probabilities and updating its estimates for car poses and landmark positions over time (with the help of incoming observations).

## Implementation
Our iSAM2 node takes as input 1.) positions of the landmarks/cones observed at the current time step from our perceptions pipeline, and 2.) odometry information. FIrst, we predict our current pose using the received odometry information. Variable node, x_{n}_ representing the car pose at the current time stamp is added alongside a factor node connecting x_{n}_ to x_{n-1}_, the variable node representing the previous car pose.

After determining the car pose, data association is performed on the cones observed at the current timestamp to determine which of the observed cones are new. To perform this data association, the Mahalanobis distance is calculated between 1 observed cone, and all iSAM2 estimates for the previously seen cones. Intuitively, the Mahalanobis distance represents how much the observed cone resembles a previously seen cone (the smaller the distance, the more the observed cone resembles the previously seen cone). If the smallest distance is greater than the Mahalanobis Distance Threshold, then the observed cone is a new cone (because it is not similar enough to any previously seen cone).

This process is repeated for all observed cones. Each detected new cone must be added to the factor graph as a variable node with a factor node connected to x_{n}_, the variable node representing the current car pose.
