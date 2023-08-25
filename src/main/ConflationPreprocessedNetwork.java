package main;

import it.unimi.dsi.fastutil.Hash;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.NetworkWriter;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.network.NetworkUtils;

import java.util.*;

/**
 * Contains network and associated data for conflation
 */
public class ConflationPreprocessedNetwork {

    private final static Logger LOG = LogManager.getLogger(ConflationPreprocessedNetwork.class);

    private Network network;
    private HashMap<Id<Node>, ConflationPreprocessedNode> preprocessedNodeMap = new HashMap<>();
    private HashSet<ConflationPreprocessedNode> terminalNodes = new HashSet<>();
    private HashMap<Long, Segment> segmentMap = new HashMap<>();
    private long segmentId = 0;
    private HashMap<Integer, HashMap<Integer, HashSet<Long>>> RTree = new HashMap<>();
    private final double bufferTolerance;
    private final double nodeTolerance;
    private final double RTreeSquareDimension;

    // These Maps allow to keep track of the links that have been cut and deleted
    private HashMap<Id<Link>, ArrayList<Double>> parentLinksCutPositionMap = new HashMap<>();   // <parentId, cutPositionList>
    private HashMap<Id<Link>, ArrayList<Link>> parentLinksChildrenMap = new HashMap<>();        // <parentId, childrenLinkList>
    private HashMap<Id<Link>, Id<Link>> childrenLinksParentIdMap = new HashMap<>();             // <childId, parentId>
    private HashMap<Id<Link>, Integer> childrenPlacementInParentMap = new HashMap<>();          // <childId, placementInParent>


    /**
     * Constructor
     * @param network Network to preprocess
     * @param bufferTolerance segment location uncertainty : half-width of segment buffer zone
     * @param nodeTolerance node location uncertainty
     * @param RTreeSquareDimension  rTree associates neighbouring segments by making a grid. Segments crossing same squarein grid placed in same rTree branch. One segment can be in several branches. rTreeSquareDimension must be > refBufferTolerance + targetBufferTolerance
     */
    public ConflationPreprocessedNetwork(Network network, double bufferTolerance, double nodeTolerance, double RTreeSquareDimension) {
        this.network = network;
        this.bufferTolerance = bufferTolerance;
        this.nodeTolerance = nodeTolerance;
        this.RTreeSquareDimension = RTreeSquareDimension;
    }

    public void preprocessNetwork(HashSet<String> modesToKeep) {
        preprocessNetwork(Math.PI/6, modesToKeep, false);
    }

    public void preprocessNetwork(HashSet<String> modesToKeep, boolean allNodesTerminal) {
        preprocessNetwork(Math.PI/6, modesToKeep, allNodesTerminal);
    }

    public void preprocessNetwork(double angleTolerance, HashSet<String> modesToKeep) {
        preprocessNetwork(angleTolerance, modesToKeep, false);
    }


    /**
     * Preprocesses network : checks every node if terminal or not
     * makes segments between terminal nodes using DFS algorithm and saves them in data structure
     * builds R-TRee
     * @param angleTolerance tolerance for link to be considered opposite direction: angle between links must be in [PI - angleTolerance, PI + angleTolerance]
     * @param modesToKeep keep in Networks only the links allowing one mode in modesToKeep
     * @param allNodesTerminal if true, forces nodes to be terminal and all segments to be only one link
     */
    public void preprocessNetwork(double angleTolerance, HashSet<String> modesToKeep,  boolean allNodesTerminal) {
        LOG.info("Preprocessing network");
        LOG.info("Looking for terminal nodes");
        for (Node node : this.network.getNodes().values()) {
            ConflationPreprocessedNode preprocessedNode = new ConflationPreprocessedNode(node, true);
            if (!allNodesTerminal) {
                preprocessedNode.computeIsTerminal(angleTolerance);
            }
            this.preprocessedNodeMap.put(node.getId(), preprocessedNode);
            if (preprocessedNode.getIsTerminal()) {
                terminalNodes.add(preprocessedNode);
            }
        }
        LOG.info("Done looking for terminal nodes");

        /* create segments : composed of all the links between two terminal nodes
        Start from one link and perform a depth first search (DFS) :
        We give the links a role of nodes as nodes can be used in several different segments, contrary to links that are only used once
        We must avoid U-turns in segments : when inside a segment, always choose the following link with the smallest angle difference
         */

        Stack<Link> viewedLinksStack = new Stack<>();
        ArrayList<Link> segmentLinks = new ArrayList<>();
        HashSet<Link> unexploredLinks = new HashSet<>(network.getLinks().values());
        for (Link link : new HashSet<>(unexploredLinks)) {
            HashSet<String> commonModes = new HashSet<>(modesToKeep);
            commonModes.retainAll(link.getAllowedModes());
            if (commonModes.isEmpty()) {
                unexploredLinks.remove(link);
            }
        }

        long nofLinks = unexploredLinks.size();

        LOG.info("Constructing segments");
        while (!unexploredLinks.isEmpty()) {
            // Initialization of first link
            Link currentLink = unexploredLinks.iterator().next();

            /// Go backwards until finding a terminal fromNode
            while (!preprocessedNodeMap.get(currentLink.getFromNode().getId()).getIsTerminal()) {
                currentLink = straightestUnexploredLink(currentLink, false, unexploredLinks);
            }

            goForward(currentLink, viewedLinksStack, segmentLinks, unexploredLinks);
            boolean lastMoveIsBackwards = false;


            while (!viewedLinksStack.isEmpty()) {
                if (preprocessedNodeMap.get(currentLink.getToNode().getId()).getIsTerminal() || lastMoveIsBackwards) {
                    // if toNode is terminal or last move is backwards
                    if (!lastMoveIsBackwards) {
                        createSegment(segmentLinks);
                    }
                    Link nextLink = nextUnexploredLink(currentLink, unexploredLinks);
                    if (nextLink != null) {
                        // if next unexplored link exists : keep going in depth
                        currentLink = goForward(nextLink, viewedLinksStack, segmentLinks, unexploredLinks);
                        lastMoveIsBackwards = false;
                    } else {
                        // Go backwards to the last viewed link with terminal toNode where it's possible to go forward to an unexplored link
                        Link previousLink = viewedLinksStack.pop();
                        while (!preprocessedNodeMap.get(previousLink.getFromNode().getId()).getIsTerminal()) {
                            previousLink = viewedLinksStack.pop();
                        }
                        // if viewedLinksStack is empty, we get out of the loop, look for other unexplored links (very unlikely to find) and start a new DFS from there
                        if (!viewedLinksStack.isEmpty()) {
                            currentLink = viewedLinksStack.peek(); // last link before the intersection (<=> terminal node)
                        }
                        lastMoveIsBackwards = true;
                    }
                } else {
                    // if node is not terminal
                    /* considering how we define a non-terminal node, the straightest link cannot have been explored before
                    (this can be false if the angle tolerance is >= 90° or pi rad)
                    we don't bother checking if it has been
                    */
                    currentLink = straightestUnexploredLink(currentLink, true, unexploredLinks);
                    goForward(currentLink, viewedLinksStack, segmentLinks, unexploredLinks);
                    lastMoveIsBackwards = false;
                }
            }
        }
        LOG.info("Done constructing segments");
        LOG.info("Done preprocessing network");

        LOG.info("Building R-Tree");
        buildRTree();
        LOG.info("Done building R-Tree");
    }


    /**
     * Used in preprocessNetwork : in DFS, explores one more link forward
     * @param link new link to explore
     * @param viewedLinksStack stack of the links algorithm has gone through to reach current location from starting link. Is used to go backwards once a branch is fully explored, links are then removed from stack.
     * @param segmentLinks List of links in the segment currently being built
     * @param unexploredLinks Set of all unexplored links in network
     * @return newly explored link
     */
    private Link goForward(Link link, Stack<Link> viewedLinksStack, ArrayList<Link> segmentLinks, HashSet<Link> unexploredLinks) {
        viewedLinksStack.push(link);
        segmentLinks.add(link);
        unexploredLinks.remove(link);
        return link;
    }


    protected static Coord getLinkVectorCoords(Link l) {
        Coord fromCoords = l.getFromNode().getCoord();
        Coord toCoords = l.getToNode().getCoord();
        double x = toCoords.getX() - fromCoords.getX();
        double y = toCoords.getY() - fromCoords.getY();
        return new Coord(x,y);
    }


    /**
     * Used in preprocessNetwork : Looks for unexplored adjacent link with the closest direction to link in parameter
     * @param link link we want to look for adjacent links
     * @param forward If true, looks at outLinks from toNode. If false, looks at inLinks to fromNode
     * @param unexploredLinks Set of unexplored links: algorithm will only consider those
     * @return link with closest direction from entry parameter link
     */
    private Link straightestUnexploredLink(Link link, boolean forward, HashSet<Link> unexploredLinks) {
        ArrayList <Link> links = new ArrayList<>();
        String nodeId = new String();
        if (forward) {
            links.addAll(link.getToNode().getOutLinks().values());
            nodeId = link.getToNode().getId().toString();
        }
        else {
            links.addAll(link.getFromNode().getInLinks().values());
            nodeId = link.getFromNode().getId().toString();
        }

        // Removing already explored links
        for (int i=links.size()-1; i>=0; i--) {
            if (!unexploredLinks.contains(links.get(i))) { links.remove(i); }
        }
        if (links.isEmpty()) {
            LOG.warn("No unexplored link found on intermediate node "+nodeId+", should be terminal");
            return null;
        }

        Coord linkCoords = getLinkVectorCoords(link);
        Link currentStraightest = links.get(0);
        double currentMaxCosine = VectOp.cosine(linkCoords, getLinkVectorCoords(currentStraightest));
        for (int i=1; i<links.size(); i++) {
            Link currentLink = links.get(i);
            double currentCosine = VectOp.cosine(linkCoords, getLinkVectorCoords(currentLink));
            if (currentCosine > currentMaxCosine) {
                currentStraightest = currentLink;
                currentMaxCosine = currentCosine;
            }
        }
        return currentStraightest;
    }


    /**
     * Used in preprocessNetwork : Chooses any unexplored link going forward
     * @param link Link one wants to go forward from
     * @param unexploredLinks Set of unexplored Links
     * @return Next link
     */
    private Link nextUnexploredLink(Link link, HashSet<Link> unexploredLinks) {
        for (Link potentialNextLink: link.getToNode().getOutLinks().values()) {
            if (unexploredLinks.contains(potentialNextLink)) {
                return potentialNextLink;
            }
        }
        return null;
    }


    /**
     * Used in preprocessNetwork, cutSegment : Creates new Segment in PreprocessedNetwork and manages its storage in data structures
     * @param links list of links composing the segment
     * @return new Segment
     */
    private long createSegment(ArrayList<Link> links) {
        ArrayList<Link> segmentLinks = new ArrayList<Link>();
        segmentLinks.addAll(links);
        segmentMap.put(segmentId, new Segment(segmentLinks, segmentId));
        segmentId++;
        links.clear();
        return segmentId-1;
    }


    /**
     * Used in cutSegment : removes segment from PreprocessedNetwork
     * @param segmentId Id of segment to be removed
     */
    private void removeSegment(Long segmentId) {
        if (segmentMap.containsKey(segmentId)) {
            segmentMap.remove(segmentId);
        } else {
            LOG.warn("Attempted to remove segment "+segmentId+" which doesn't exist in network\"");
        }
    }


    /**
     * Used in preprocessNetwork : Builds R-Tree and classifies segments in branches
     * R-Tree associates neighbouring segments by making a grid: Segments crossing same square in grid placed in same branch.
     * One segment can be in several branches
     */
    private void buildRTree() {
        for (Segment targetSegment : this.getSegments().values()) {
            addOneSegmentInRTree(targetSegment);
        }
    }


    /**
     * Used in buildRTree, cutSegment : Checks the branches of R-Tree for one segment
     * @param targetSegment segment to add in R-Tree
     */
    private void addOneSegmentInRTree(Segment targetSegment) {
        Coord fromCoord = targetSegment.getFromNode().getCoord();
        Coord toCoord = targetSegment.getToNode().getCoord();
        Coord vectorCoord = targetSegment.getCoord();
        if (vectorCoord.getX() == 0) {
            int i = (int) (fromCoord.getX()/RTreeSquareDimension);
            int jMin = (int) ((Math.min(fromCoord.getY(), fromCoord.getY()+vectorCoord.getY()) - bufferTolerance) / RTreeSquareDimension);
            int jMax = (int) ((Math.max(fromCoord.getY(), fromCoord.getY()+vectorCoord.getY()) + bufferTolerance) / RTreeSquareDimension);
            for (int j = jMin; j <= jMax; j++) {
                addSegmentToRTreeBranch(i, j, targetSegment);
            }
        } else {
            Coord unitVector = VectOp.getUnitVector(vectorCoord);
            int iInit = (int) ((fromCoord.getX() - this.bufferTolerance * unitVector.getX()) / RTreeSquareDimension);
            int iFinal = (int) ((toCoord.getX() + this.bufferTolerance * unitVector.getX()) / RTreeSquareDimension);
            int iMin = Math.min(iInit, iFinal);
            int iMax = Math.max(iInit, iFinal);
            for (int i = iMin; i <= iMax; i++) {
                double y1 = fromCoord.getY() + vectorCoord.getY() * (RTreeSquareDimension*i - fromCoord.getX()) / vectorCoord.getX();
                double y2 = fromCoord.getY() + vectorCoord.getY() * (RTreeSquareDimension*(i+1) - fromCoord.getX()) / vectorCoord.getX();
                int jMin = (int) ((Math.min(y1,y2) - this.bufferTolerance) / RTreeSquareDimension);
                int jMax = (int) ((Math.max(y1,y2) + this.bufferTolerance) / RTreeSquareDimension);
                for (int j = jMin; j <= jMax; j++) {
                    addSegmentToRTreeBranch(i, j, targetSegment);
                }
            }
        }
    }


    /**
     * Used in addOneSegmentInRTree : Adds segment to R-Tree branch, identified by 2 integers (i,j) and adds branch to segment branch List
     * @param i branch first id
     * @param j branch second id
     * @param targetSegment segment to add to (i,j) branch
     */
    private void addSegmentToRTreeBranch(int i, int j, Segment targetSegment) {
        if (!this.RTree.containsKey(i)) {
            this.RTree.put(i, new HashMap<>());
        }
        if (!this.RTree.get(i).containsKey(j)) {
            this.RTree.get(i).put(j, new HashSet<>());
        }
        this.RTree.get(i).get(j).add(targetSegment.getId());
        targetSegment.addRTreeBranch(new BidimensionalIndex(i,j));
    }


    public double getBufferTolerance() {
        return this.bufferTolerance;
    }

    public double getNodeTolerance() {
        return this.nodeTolerance;
    }

    protected HashMap<Integer, HashMap<Integer, HashSet<Long>>> getRTree() {
        return this.RTree;
    }


    /**
     * Used in saveSimplifiedNetwork : builds new network with segments as links
     * @return simplified network
     */
    public Network createSimplifiedNetworkFromSegments(){
        LOG.info("Creating simplified network");
        LOG.info("Creating nodes");
        Network simplifiedNet = NetworkUtils.createNetwork();
        // creation of nodes
        for (ConflationPreprocessedNode node : this.preprocessedNodeMap.values()) {
            if (node.getIsTerminal()) {
                NetworkUtils.createAndAddNode(simplifiedNet,node.getNode().getId(), node.getNode().getCoord());
                // simplifiedNet.getFactory().createNode(node.getNode().getId(), node.getNode().getCoord());
            }
        }
        LOG.info("Done creating nodes");

        // creation of links
        LOG.info("Creating simplified links");
        for (Segment segment : this.segmentMap.values()) {
            ArrayList<Link> links = segment.getLinks();

            // Setting capacity as the minimum of all link capacities
            // Setting freespeed as the average of the links freeSpeed and length as the sum of all lengths
            // Setting number of lanes as the number of lanes in the last link
            double capacity = segment.getLinks().get(0).getCapacity();
            double totalTime = 0;
            double totalLength = 0;
            for (Link link : segment.getLinks()) {
                if (link.getCapacity() < capacity) { capacity = link.getCapacity(); }
                totalTime += link.getLength()/link.getFreespeed();
                totalLength += link.getLength();
            }

            Link l = NetworkUtils.createAndAddLink(simplifiedNet, Id.createLinkId(segment.getId()), segment.getFromNode(), segment.getToNode(),
                    totalLength,totalLength/totalTime, capacity, segment.getLinks().get(segment.getLinks().size()-1).getNumberOfLanes());
            // Link l = simplifiedNet.getFactory().createLink(Id.createLinkId(segment.getId()), segment.getFromNode(), segment.getToNode());

            // Setting allowed modes
            HashSet<String> allowedModes = new HashSet<String>();
            for (String mode : segment.getLinks().get(0).getAllowedModes()) {
                boolean allowed = true;
                for (Link link : segment.getLinks()) {
                    if (!link.getAllowedModes().contains(mode)) {
                        allowed = false;
                        break;
                    }
                }
                if (allowed) { allowedModes.add(mode); }
            }
            l.setAllowedModes(allowedModes);
        }
        LOG.info("Done creating simplified links");
        LOG.info("Done creating simplified network");
        return simplifiedNet;
    }


    /**
     * to visualize if segments were made correctly : creates and saves new network with segments as links
     * @param path where simplified Network is saved (as .xml file)
     */
    protected void saveSimplifiedNetwork(String path) {
        Network simplifiedNet = createSimplifiedNetworkFromSegments();
        new NetworkWriter(simplifiedNet).write(path);
    }

    public HashMap<Long,Segment> getSegments() {
        return segmentMap;
    }

    public HashMap<Id<Node>, ConflationPreprocessedNode> getPreprocessedNodes() {
        return preprocessedNodeMap;
    }


    /**
     * Cuts Segment into 2 new segments
     * @param segmentId Segment to be cut
     * @param cutLinkIndex Link on which the cut is to be done
     * @param cutPosition in [0,1]: position in link where cut is to be done. If in ]0,1[, link cut into 2 new links. Else (<=> in {0,1}), no new link made & segment cut on the joint between 2 links
     * @return List of newly created Segment ids
     */
    public ArrayList<Long> cutSegment(Long segmentId, int cutLinkIndex, double cutPosition) {
        Segment segment  = segmentMap.get(segmentId);
        removeSegment(segmentId);
        ArrayList<Long> newSegmentIdsList = new ArrayList<>();
        ArrayList<Link> newSegmentLinksList1 = new ArrayList<>();
        ArrayList<Link> newSegmentLinksList2 = new ArrayList<>();

        if (cutPosition == 0 || cutPosition == 1) {
            // Creating new segment link lists
            if (cutPosition == 1) {
                cutLinkIndex += 1;
            }

            newSegmentLinksList1.addAll(segment.getLinks().subList(0, cutLinkIndex));
            newSegmentLinksList2.addAll(segment.getLinks().subList(cutLinkIndex, segment.getLinks().size()));
        } else {
            // Creating new node and links
            Coord point = segment.segmentPointCoord(cutLinkIndex, cutPosition);
            Node newNode = createNodeWithRandomId(point);

            Id<Link> newLinkId1 = createRandomLinkId();
            Id<Link> newLinkId2 = createRandomLinkId();

            Link link = segment.getLinks().get(cutLinkIndex);
            Node fromNode = link.getFromNode();
            Node toNode = link.getToNode();
            double length1 = link.getLength() * cutPosition;
            double length2 = link.getLength() * (1-cutPosition);

            Link newLink1 = NetworkUtils.createLink(newLinkId1, fromNode, newNode, network,
                    length1, link.getFreespeed(), link.getCapacity(), link.getNumberOfLanes());
            newLink1.setAllowedModes(link.getAllowedModes());
            Link newLink2 = NetworkUtils.createLink(newLinkId2, newNode, toNode, network,
                    length2, link.getFreespeed(), link.getCapacity(), link.getNumberOfLanes());
            newLink2.setAllowedModes(link.getAllowedModes());

            // Removing old link and adding new ones
            fromNode.removeOutLink(link.getId());
            toNode.removeInLink(link.getId());
            network.removeLink(link.getId());

            // Keeping track of the relation between deleted parent link and children links
            int parentPlacement = 0;
            if (childrenLinksParentIdMap.containsKey(link)) {
                Link grandParentLink = network.getLinks().get(childrenLinksParentIdMap.get(link));
                parentPlacement = childrenPlacementInParentMap.get(link.getId());
                double newCutPosition = getPositionInParent(link.getId(), cutPosition);
                parentLinksCutPositionMap.get(grandParentLink).add(parentPlacement+1, newCutPosition);
                parentLinksChildrenMap.get(grandParentLink.getId()).set(parentPlacement, newLink1);
                parentLinksChildrenMap.get(grandParentLink.getId()).add(parentPlacement+1, newLink2);

                childrenLinksParentIdMap.put(newLinkId1, grandParentLink.getId());
                childrenLinksParentIdMap.put(newLinkId2, grandParentLink.getId());
            } else {
                parentLinksCutPositionMap.put(link.getId(), new ArrayList<>());
                parentLinksCutPositionMap.get(link.getId()).add(cutPosition);
                parentLinksChildrenMap.put(link.getId(), new ArrayList<>());
                parentLinksChildrenMap.get(link.getId()).add(newLink1);
                parentLinksChildrenMap.get(link.getId()).add(newLink2);

                childrenLinksParentIdMap.put(newLinkId1, link.getId());
                childrenLinksParentIdMap.put(newLinkId2, link.getId());
            }
            childrenPlacementInParentMap.put(newLinkId1, parentPlacement);
            childrenPlacementInParentMap.put(newLinkId1, parentPlacement+1);

            addLinkToNetwork(newLink1);
            addLinkToNetwork(newLink2);

            // Creating new segment link lists
            newSegmentLinksList1.addAll(segment.getLinks().subList(0, cutLinkIndex));
            newSegmentLinksList1.add(newLink1);

            newSegmentLinksList2.add(newLink2);
            newSegmentLinksList2.addAll(segment.getLinks().subList(cutLinkIndex+1, segment.getLinks().size()));
        }

        newSegmentIdsList.add(createSegment(newSegmentLinksList1));
        newSegmentIdsList.add(createSegment(newSegmentLinksList2));

        for (long id : newSegmentIdsList) {
            Segment newSegment = segmentMap.get(id);
            addOneSegmentInRTree(newSegment);
        }

        return newSegmentIdsList;
    }


    /**
     * Creates new node in this.network with random Id
     * @param nodeLocation Coordinates of node to be created
     * @return new Node
     */
    private Node createNodeWithRandomId(Coord nodeLocation) {
        // TODO Méthode provisoire pour créer un id qui n'existe pas dans le réseau
        Id<Node> newId = Id.createNodeId((long) (Math.random()*2000000000));
        while (network.getNodes().containsKey(newId)) {
            newId = Id.createNodeId((long) (Math.random()*2000000000));
        }
        Node newNode = NetworkUtils.createNode(newId, nodeLocation);
        network.addNode(newNode);
        return newNode;
    }


    private Id<Link> createRandomLinkId() {
        Id<Link> newId = Id.createLinkId((long) (Math.random()*2000000000));
        while (network.getLinks().containsKey(newId)) {
            newId = Id.createLinkId((long) (Math.random()*2000000000));
        }
        return newId;
    }


    private void addLinkToNetwork(Link link) {
        link.getFromNode().addOutLink(link);
        link.getToNode().addInLink(link);
        network.addLink(link);
    }


    /**
     * Used in cutSegment : Gives position of a point in parent link from position in child link
     * Position is in [0,1], 0 being fromNode, 1 being toNode
     * parentLink : link in original Network
     * child link : link in current network resulting from cuts in parentLink
     * @param childId id of childLink
     * @param position position of point in childLink
     * @return position of same point in parentLink
     */
    private double getPositionInParent(Id<Link> childId, double position) {
        int childPlacement = childrenPlacementInParentMap.get(childId);
        double initCut = parentLinksCutPositionMap.get( childrenLinksParentIdMap.get(childId) ).get(childPlacement);
        double finalCut = parentLinksCutPositionMap.get( childrenLinksParentIdMap.get(childId) ).get(childPlacement+1);
        return position * (finalCut - initCut) + initCut;
    }


    /**
     * Prints Link id and, if link results from cut, position of fromNode and toNode in parent link
     * @param linkId id of Link to be printed
     */
    public void printLink(Id<Link> linkId) {
        System.out.print(linkId.toString());
        if (childrenLinksParentIdMap.containsKey(linkId)) {
            Id<Link> parentId = childrenLinksParentIdMap.get(linkId);
            int childPlacement = childrenPlacementInParentMap.get(linkId);
            System.out.print(" Parent: "+parentId+", cut between: ["+parentLinksCutPositionMap.get(childPlacement)+", "+parentLinksCutPositionMap.get(childPlacement+1)+"]");
        }
        System.out.print("; ");
    }
}