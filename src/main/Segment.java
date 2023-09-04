package main;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;

import java.util.ArrayList;
import java.util.HashSet;


/**
 * Elementary object of ConflationPreprocessedNetwork : contains several links, starts and ends on terminal nodes,
 * Inbetween nodes are all intermediate nodes.
 */
public class Segment {
    private ArrayList<Link> links;
    private final long id;
    private HashSet<BidimensionalIndex> RTreeBranchesSet = new HashSet<>();

    /**
     * Constructor
     * @param links List of links contained in Segment
     * @param id id given to the Segment
     */
    public Segment(ArrayList<Link> links, long id) {
        this.id = id;
        this.links = links;
    }

    public Node getFromNode() {
        return links.get(0).getFromNode();
    }

    public Node getToNode() {
        return links.get(this.links.size()-1).getToNode();
    }

    public ArrayList<Node> getAllNodes() {
        ArrayList<Node> nodes = new ArrayList<Node>();
        nodes.add(this.links.get(0).getFromNode());
        for (Link link: this.links) {
            nodes.add(link.getToNode());
        }
        return nodes;
    }

    /**
     * @return All nodes except fromNode and toNode
     */
    public ArrayList<Node> getIntermediateNodes() {
        ArrayList<Node> nodes = new ArrayList<Node>();
        for (Link link: this.links) {
            nodes.add(link.getToNode());
        }
        nodes.remove(nodes.size()-1);
        return nodes;
    }

    public ArrayList<Link> getLinks() {
        return links;
    }

    public long getId() {
        return id;
    }


    /**
     * @return Coords of the vector going from fromNode to toNode
     */
    public Coord getCoord() {
        Coord fromCoord = this.getFromNode().getCoord();
        Coord toCoord = this.getToNode().getCoord();
        return VectOp.addVectors(toCoord, VectOp.extPdt(-1,fromCoord));
    }


    /**
     * @return straight line distance between fromNode and toNode
     */
    public double getStraightLineLength() {
        return VectOp.length(this.getCoord());
    }

    public LocalizedVector getLocalizedVector() {
        return new LocalizedVector(getFromNode().getCoord(), getCoord());
    }

    public Coord getSegmentUnitVector() {
        return VectOp.getUnitVector(this.getCoord());
    }


    /**
     * Finds closest Segment point from parameter point
     * @param point coordinates of the point one wants to find closest Segment point from (parameter point)
     * @param cutLinkIndexWrapper Can initially have any value, when function is executed, gets placement in Segment of the link the closest point belongs to
     * @return position of closest point in link indicated by value contained in cutLinkIndexWrapper (position: between 0 = fromNode and 1 = toNode)
     */
    public double closestPointInSegment(Coord point, ArrayList<Integer> cutLinkIndexWrapper) {
        // Finding which node of the links of the segments in this Polyline is the closest to point
        int minimalLinkToNode = -1;
        double minimalExistingNodeDistance = VectOp.distance(getFromNode().getCoord(), point);
        for (int i = 0; i < getLinks().size(); i++) {
            double currentDistance = VectOp.distance(getLinks().get(i).getToNode().getCoord(), point);
            if (currentDistance < minimalExistingNodeDistance) {
                minimalLinkToNode = i;
                minimalExistingNodeDistance = currentDistance;
            }
        }

        // Analysing the respective closest point on the 2 links right before and right after the closest node
        double cutPos = 0;
        int cutLinkIndex = 0;
        if (minimalLinkToNode == -1) {
            Link link = getLinks().get(0);
            cutPos = (new LocalizedVector(link)).closestPointInLocalizedVector(point);
        } else {
            Link link1 = getLinks().get(minimalLinkToNode);
            if (minimalLinkToNode == getLinks().size()-1) {
                cutPos = (new LocalizedVector(link1)).closestPointInLocalizedVector(point);
                cutLinkIndex = minimalLinkToNode;
            } else {
                Link link2 = getLinks().get(minimalLinkToNode+1);
                double pos1 = (new LocalizedVector(link1)).closestPointInLocalizedVector(point);
                double pos2 = (new LocalizedVector(link2)).closestPointInLocalizedVector(point);
                Coord point1 = VectOp.addVectors(VectOp.extPdt(1-pos1, link1.getFromNode().getCoord()), VectOp.extPdt(pos1, link1.getToNode().getCoord()));
                Coord point2 = VectOp.addVectors(VectOp.extPdt(1-pos2, link2.getFromNode().getCoord()), VectOp.extPdt(pos2, link2.getToNode().getCoord()));
                if (VectOp.distance(point, point2) < VectOp.distance(point, point1)) {
                    cutLinkIndex = minimalLinkToNode;
                    cutPos = pos1;
                } else {
                    cutLinkIndex = minimalLinkToNode+1;
                    cutPos = pos2;
                }
            }
        }
        cutLinkIndexWrapper.clear();
        cutLinkIndexWrapper.add(cutLinkIndex);
        return cutPos;
    }


    /**
     * Gives the coordinates of point belonging to Segment indicated by the link it belongs to and its position in said link
     * @param index placement of link in Segment
     * @param pos position of point in link (between 0 = fromNode and 1 = toNode)
     * @return Coords of the point of Segment indicated by index and pos
     */
    public Coord segmentPointCoord(int index, double pos) {
        Link link = this.getLinks().get(index);
        return VectOp.addVectors(VectOp.extPdt(1-pos, link.getFromNode().getCoord()), VectOp.extPdt(pos, link.getToNode().getCoord()));
    }


    /**
     * Computes the distance between Segment and point
     * @param point Coordinates of point one wants the distance with Segment
     * @return Shortest distance between point and Segment
     */
    public double distanceWithPoint(Coord point) {
        ArrayList<Integer> linkIndex = new ArrayList<>();
        double pos = closestPointInSegment(point, linkIndex);
        return VectOp.distance(point, segmentPointCoord(linkIndex.get(0), pos));
    }


    /**
     * Adds a branch in the Set of R-Tree branches Segment belongs to
     * @param branch branch to add
     */
    public void addRTreeBranch(BidimensionalIndex branch) {
        this.RTreeBranchesSet.add(branch);
    }

    public HashSet<BidimensionalIndex> getRTreeBranchesSet() {
        return this.RTreeBranchesSet;
    }
}
