package main;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.network.Link;

import java.util.ArrayList;

public class LocalizedVector {
    private Coord originNode;
    private Coord vector;

    LocalizedVector(Link link) {
        this.originNode = link.getFromNode().getCoord();
        this.vector = VectOp.addVectors(link.getToNode().getCoord(), VectOp.extPdt(-1, originNode));
    }


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

    public void translateWholeVector(Coord newFromCoord) {
        this.originNode = newFromCoord;
    }

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

    public LocalizedVector expressInOrthoNBase(Coord origin, Coord u) {
        Coord originNode = VectOp.addVectors(this.originNode, VectOp.extPdt(-1, origin));
        originNode = VectOp.expressInOrthoNBase(originNode, u);
        Coord vector = VectOp.expressInOrthoNBase(this.vector, u);
        return new LocalizedVector(originNode, vector);
    }


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
