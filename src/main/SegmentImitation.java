package main;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.network.Node;

import java.util.ArrayList;

public class SegmentImitation {
    Node fromNode;
    Node toNode;

    public SegmentImitation(Node fromNode, Node toNode) {
        this.fromNode = fromNode;
        this.toNode = toNode;
    }

    public Node getFromNode() {
        return this.fromNode;
    }

    public Node getToNode() {
        return this.toNode;
    }

    public Coord getCoord() {
        return VectOp.addVectors(toNode.getCoord(), VectOp.extPdt(-1, fromNode.getCoord()));
    }

    public ArrayList<Coord> getBufferBorder(double tolerance, double leftOffset) {
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

        Coord departurePoint = VectOp.addVectors(VectOp.addVectors(
                fromNodeCoord, VectOp.extPdt(-tolerance, unitVector)), offset);
        Coord borderVector = VectOp.addVectors(segmentVector, VectOp.extPdt(2*tolerance,unitVector));

        ArrayList<Coord> bufferBorder = new ArrayList<>();
        bufferBorder.add(departurePoint);
        bufferBorder.add(borderVector);
        return bufferBorder;
    }
}
