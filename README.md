# matsim-network-conflation

## What does this code do ?

##### This code was made during a Master first year internship in research.
##### In order to represent road networks in MATSim, several datasets from different sources could be useful to improve the reliability of the data. Starting from two overlapping MATSim networks this code is part of what is necessary to conflate them into one.
#### **Input data:**
##### Two MATSim networks to be conflated: a reference network and a target network
#### **Description of the algorithm:**
##### For each network, an algorithm builds segments from the links. If the option 'allNodesAreTerminal' is set to True, the all segments will be composed of only one link. Otherwise, the algorithm will associate all consecutive links between two intersections into one segment.
![image](https://github.com/NoeFillon/matsim-network-conflation/assets/141751382/b029599e-5a64-4a51-b1c0-11adfb6af334)
##### In the figure above, a segment will associate all the links between two nodes displayed in red.
##### For each segment of the reference network, the an algorithm builds all the polylines coresponding to this segment from segments of the target network. Target polylines (built from segment of target network) can be constructed by associating segments together and/or by cutting them. Compatible target polylines are kept and are associated a similarity score based on how close they are geometrically to the reference segment.
#### **Output:**
##### The output goes into the networkCandidates HashMap which contains, for each segment Id from the reference network, a set of compatible target polylines with an associated score
