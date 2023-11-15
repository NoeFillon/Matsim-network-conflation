# matsim-network-conflation

## What does this code do ?

##### This code was made during a Master first year internship in research.
##### In order to represent road networks in MATSim, several datasets from different sources could be useful to improve the reliability of the data. Starting from two overlapping MATSim networks this code is part of what is necessary to conflate them into one.

#### **Input data:**
##### Two MATSim networks to be conflated: a reference network and a target network

#### **Description of the algorithm:**
##### For each network, an algorithm builds segments from the links. If the option `allNodesAreTerminal` is set to true, the all segments will be composed of only one link. Otherwise, the algorithm will associate all consecutive links between two intersections into one segment.
![image](https://github.com/NoeFillon/matsim-network-conflation/assets/141751382/b029599e-5a64-4a51-b1c0-11adfb6af334)
##### In the figure above, a segment will associate all the links between two nodes displayed in red.
##### For each segment of the reference network, the an algorithm builds all the polylines coresponding to this segment from segments of the target network. Target polylines (built from segment of target network) can be constructed by associating segments together and/or by cutting them. Compatible target polylines are kept and are associated a similarity score based on how close they are geometrically to the reference segment.
##### As a more detailed explanation of the code, here is a flow chart of the code. Only the main function are represented and their colour represents the class they belong to. An arrow means the function pointed by the arrow (to the right) calls the function the arrow staarts from (to the left).
![image](https://github.com/NoeFillon/matsim-network-conflation/assets/141751382/1657cef1-7e2e-4b1c-b5a3-3094630bd1ad)

#### **Output:**
##### The output goes into the networkCandidates HashMap which contains, for each segment Id from the reference network, a set of compatible target polylines with an associated score

#### **Continuation perspective:**
##### This code does not completely perform the task of conflating the networks into one. Future works should start from this code's output and associate each reference segment to a target polyline, optimizing the total score of the selected associations un the constraint of making sure the assciations are topologically compatible (for example, two different reference segments cannot be associated to two polylines with a common section).

## How to use this code ?

##### Execute src/main/Main class. It contains two instructions:
    NetworkConflator conflator = new NetworkConflator(String refPath, double refNodeTolerance, double refBufferTolerance,
                                        boolean allRefNodesTerminal, String targetPath, double targetNodeTolerance,
                                        double targetBufferTolerance, boolean allTargetNodesTerminal,
                                        double rTreeSquareDimension, HashSet<String> modesToKeep, boolean savePreprocessedNetworks,
                                        String simplifiedRefNetworkSavingPath, String simplifiedTargetNetworkSavingPath)
##### This function preprocesses the two networks and creates an object NetworkConflator that contains the functions necessary for the next steps.
##### If one network is significantly less detailed than the other, we recommend taking the least detailed one as reference network.
##### The parameter `allNodesAreTerminal` (simplifying the network into segments if set to `false`) should be set depending on the precision of both networks. A network should be simplified (`allNodesAreTerminal = false`) only if it does not increase the gap in precision between the two networks. Therefore, if there is a difference in precision/exhaustivity, reference network should generally not be simplified. Generally, if a network is exhaustive, it can be simplified to increase performance.
##### Default tolerance values have been chosen in the code, they should be defined regarding the geometrical precision of the networks.
##### `RTreeSquareDimension` must be as small as possible to increase performance but bigger than `refBufferTolerance + targetBufferTolerance`.
##### More details about the parameters in function documentation.
##### The second instruction is:
    HashMap<Long, HashSet<ScoredPolyline>> networkCandidates = conflator.populateForNetwork();
##### This instruction performs the construction of compatible target polylines associated with reference segments and computes each polyline's similarity score.
