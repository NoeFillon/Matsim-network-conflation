package main;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.network.Link;

import java.util.ArrayList;

/**
 * This class represents a vector with an origin point :
 * Contains 2 coordinates: these of the origin point and these of the vector
 */
public class LocalizedVector {
    private Coord originNode;
    private Coord vector;

    /**
     * Constructor for a LocalizedVector constructed from a link
     * @param link link of which one wants to build LocalizedVector
     */
    LocalizedVector(Link link) {
        this.originNode = link.getFromNode().getCoord();
        this.vector = VectOp.addVectors(link.getToNode().getCoord(), VectOp.extPdt(-1, originNode));
    }


    /**
     * Constructir for a LocalizedVector directly defined with origin node and vector
     * @param originNode coordinates of LocalizedVector origin node
     * @param vector coordinates of LocalizedVector vector
     */
    LocalizedVector(Coord originNode, Coord vector) {
        this.originNode = originNode;
        this.vector = vector;
    }

    public Coord getFromCoord() {
        return originNode;
    }

    public Coord getToCoord() {
        return VectOp.addVectors(originNode, vector);
    }

    public Coord getVectorCoord() {
        return vector;
    }

    public void setFromCoord(Coord fromCoord) {
        this.vector = VectOp.addVectors(VectOp.addVectors(vector, this.originNode), VectOp.extPdt(-1, fromCoord));
        this.originNode = fromCoord;
    }


    /**
     * Makes a translation of origin node without changing vector coordinates => whole vector translation
     * @param newFromCoord new coordinates for origin node
     */
    public void translateWholeVector(Coord newFromCoord) {
        this.originNode = newFromCoord;
    }


    /**
     * Buffer is a rectangle around a link or segment, used to compute overlapArea to get to the similarity between links and segments
     * This function builds left or right side of buffer area
     * @param leftOffset left/right offset of the buffer side from central LocalizedVector: >0 -> left offset ; <0 -> right offset
     * @param fromNodeProlong forward/backwards offset of the origin node of buffer border before fromNode
     * @param toNodeProlong forward/backwards offset of the final node of buffer border after toNode
     * @return LocalizedVector representing buffer border
     */
    public LocalizedVector getBufferBorder(double leftOffset, double fromNodeProlong, double toNodeProlong) {
        /*
        Return one side of the buffer border
        Input : leftRightOffset : offset of the returned segment :
            + corresponds to a distance to the left of the segment
            -, to the right
         */
        Coord unitVector = VectOp.getUnitVector(vector);
        Coord fromNodeCoord = this.originNode;
        Coord offset = VectOp.extPdt(leftOffset, VectOp.piRotation(unitVector));

        Coord departurePoint = VectOp.addVectors(fromNodeCoord, VectOp.addVectors(offset, VectOp.extPdt(-fromNodeProlong, unitVector)));
        Coord borderVector = VectOp.addVectors(vector, VectOp.extPdt(fromNodeProlong+toNodeProlong, unitVector));

        return new LocalizedVector(departurePoint, borderVector);
    }


    /**
     * Expresses this LocalizedVector in another direct orthonormal base
     * @param origin origin of target base
     * @param u x vector of target base (y vector is calculated 90° left from this vector)
     * @return new expression of LocalizedVector in target base
     */
    public LocalizedVector expressInOrthoNBase(Coord origin, Coord u) {
        Coord originNode = VectOp.addVectors(this.originNode, VectOp.extPdt(-1, origin));
        originNode = VectOp.expressInOrthoNBase(originNode, u);
        Coord vector = VectOp.expressInOrthoNBase(this.vector, u);
        return new LocalizedVector(originNode, vector);
    }


    /**
     * Finds closest point in LocalizedVector from parameter point
     * @param point parameter point one wants to find closest point in polyline from
     * @return position in LocalizedVector of closest point (position: between 0 = fromNode and 1 = toNode)
     */
    public double closestPointInLocalizedVector(Coord point) {
        Coord OP = VectOp.addVectors(point, VectOp.extPdt(-1, originNode));
        double t = VectOp.dot(OP, vector) / VectOp.dot(vector, vector);
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }
        return t;
    }
}
