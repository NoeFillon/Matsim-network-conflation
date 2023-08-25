package main;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;

import java.util.ArrayList;
import java.util.HashSet;

public class Segment {
    private ArrayList<Link> links;
    private final long id;
    private HashSet<BidimensionalIndex> RTreeBranchesSet = new HashSet<>();

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

    public Coord getCoord() {
        Coord fromCoord = this.getFromNode().getCoord();
        Coord toCoord = this.getToNode().getCoord();
        return VectOp.addVectors(toCoord, VectOp.extPdt(-1,fromCoord));
    }

    public double getStraightLineLength() {
        return VectOp.length(this.getCoord());
    }

    public LocalizedVector getLocalizedVector() {
        return new LocalizedVector(getFromNode().getCoord(), getCoord());
    }

    public Coord getSegmentUnitVector() {
        return VectOp.getUnitVector(this.getCoord());
    }

    public ArrayList<Coord> getBufferBorder(double tolerance, double leftOffset, boolean fromNodeProlong, boolean toNodeProlong) {
        /*
        Return one side of the buffer border
        Output : ArrayList<Coord> : contains departure point, vector to endpoint
        Input : leftRightOffset : offset of the returned segment :
            +1 corresponds to a distance of tolerance to the left of the segment
            -1, to the right
         */
        Coord segmentVector = this.getCoord();
        Coord unitVector = VectOp.getUnitVector(segmentVector);
        Coord fromNodeCoord = this.getFromNode().getCoord();
        Coord offset = VectOp.extPdt(leftOffset, VectOp.piRotation(unitVector));

        Coord departurePoint = VectOp.addVectors(fromNodeCoord, offset);
        if (fromNodeProlong) {
            departurePoint = VectOp.addVectors(departurePoint, VectOp.extPdt(-tolerance, unitVector));
        }
        int nOfProlong = 0;
        if (fromNodeProlong) {
            nOfProlong+=1;
        } if (toNodeProlong) {
            nOfProlong+=1;
        }
        Coord borderVector = VectOp.addVectors(segmentVector, VectOp.extPdt(nOfProlong*tolerance,unitVector));

        ArrayList<Coord> bufferBorder = new ArrayList<>();
        bufferBorder.add(departurePoint);
        bufferBorder.add(borderVector);
        return bufferBorder;
    }

    public double closestPointInSegment(Coord point, ArrayList<Integer> cutLinkIndexWrapper) {
        // cutLinkIndexWrapper can be any list, it's cleaned anyway
        // It's used to return the link index, because it's impossible to return 2 values

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


    public Coord segmentPointCoord(int index, double pos) {
        Link link = this.getLinks().get(index);
        return VectOp.addVectors(VectOp.extPdt(1-pos, link.getFromNode().getCoord()), VectOp.extPdt(pos, link.getToNode().getCoord()));
    }

    public double distanceWithPoint(Coord point) {
        ArrayList<Integer> linkIndex = new ArrayList<>();
        double pos = closestPointInSegment(point, linkIndex);
        return VectOp.distance(point, segmentPointCoord(linkIndex.get(0), pos));
    }

    public void addRTreeBranch(BidimensionalIndex branch) {
        this.RTreeBranchesSet.add(branch);
    }

    public HashSet<BidimensionalIndex> getRTreeBranchesSet() {
        return this.RTreeBranchesSet;
    }

    public void print() {
        System.out.print("Segment "+this.id);
        for (Link link : this.links) {

        }
    }
}
