package main;

import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;

import java.util.ArrayList;
import java.util.HashSet;

/**
 * Object composed containing a list of adjacent segments
 * First and last segments can be cut at some point before their end
 * Polyline therefore contains cut objects (initCut and finalCut)
 */
public class Polyline {
    private ArrayList<Segment> segments;
    private Cut initialCut;
    private Cut finalCut;

    /**
     * Constructor
     * @param segments List of the Polyline's segments
     */
    Polyline(ArrayList<Segment> segments) {
        this.segments = segments;
        this.initialCut = new Cut(0, false, 0);
        this.finalCut = new Cut(segments.get(segments.size()-1).getLinks().size()-1, false, 1);
    }

    /**
     * Constructor in case there are cuts
     * @param segments List of the Polyline's segments
     * @param initialCut cut on the origin point of the Polyline
     * @param finalCut cut on the end point of the Polyline
     */
    Polyline(ArrayList<Segment> segments, Cut initialCut, Cut finalCut) {
        this.segments = segments;
        this.initialCut = initialCut;
        this.finalCut = finalCut;
    }


    public ArrayList<Segment> getSegments() {
        return this.segments;
    }


    public Cut getInitialCut() {
        return initialCut;
    }


    public Cut getFinalCut() {
        return finalCut;
    }


    /**
     * @return origin node of the first segment regardless of initialCut
     */
    public Node getFromNodeWithoutCut() {
        return segments.get(0).getFromNode();
    }


    /**
     * @return end node of the last segment regardless of finalCut
     */
    public Node getToNodeWithoutCut() {
        return segments.get(segments.size()-1).getToNode();
    }

    /**
     * @return coordinates of the origin point of the Polyline taking initialCut into account
     */
    public Coord getFromCoordWithCut() {
        Link initialLink = segments.get(0).getLinks().get(initialCut.getCutLinkIndex());
        Coord fromNodeCoord = initialLink.getFromNode().getCoord();
        if (initialCut.getIsLinkCut()) {
            double coef = initialCut.getCutPosition();
            Coord toNodeCoord = initialLink.getToNode().getCoord();
            return VectOp.addVectors(VectOp.extPdt(1-coef, fromNodeCoord) , VectOp.extPdt(coef, toNodeCoord));
        }
        return fromNodeCoord;
    }

    /**
     * @return coordinates of the end point of the Polyline taking finalCut into account
     */
    public Coord getToCoordWithCut() {
        Link finalLink = segments.get(segments.size()-1).getLinks().get(finalCut.getCutLinkIndex());
        Coord toNodeCooord = finalLink.getToNode().getCoord();
        if (finalCut.getIsLinkCut()) {
            double coef = finalCut.getCutPosition();
            Coord fromNodeCoord = finalLink.getFromNode().getCoord();
            return VectOp.addVectors(VectOp.extPdt(1-coef, fromNodeCoord) , VectOp.extPdt(coef, toNodeCooord));
        }
        return toNodeCooord;
    }


    /**
     * A List of lists of Localized Vectors (origin point + vector) :
     * Each element of the outer List (inner lists) represents a segment containing links
     * Each element of the inner Lists represents a Link
     * @return list of lists of Localized Vectors representing the Polyline's links
     */
    public ArrayList<ArrayList<LocalizedVector>> getLinkLocalizedVectorsWithCuts() {
        ArrayList<ArrayList<LocalizedVector>> links = new ArrayList<>();
        for (int segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {
            Segment segment = segments.get(segmentIndex);
            links.add(new ArrayList<>());
            for (int linkIndex = 0; linkIndex < segment.getLinks().size(); linkIndex++) {
                Link link = segment.getLinks().get(linkIndex);
                Coord fromCoord = link.getFromNode().getCoord();
                Coord toCoord = link.getToNode().getCoord();
                boolean toAdd = false;
                boolean initialCondition = segmentIndex==0 && initialCut.getIsLinkCut() && linkIndex <= initialCut.getCutLinkIndex();
                boolean finalCondition = segmentIndex==segments.size()-1 && finalCut.getIsLinkCut() && linkIndex >= finalCut.getCutLinkIndex();
                if (!initialCondition && !finalCondition) {
                    toAdd = true;
                } else if (initialCut.getCutPosition() < finalCut.getCutPosition()) {
                    if (initialCondition && initialCut.getCutLinkIndex() == linkIndex && initialCut.getCutPosition() < 1) {
                        toAdd = true;
                        double pos = initialCut.getCutPosition();
                        fromCoord = VectOp.addVectors(VectOp.extPdt(pos, toCoord), VectOp.extPdt(1-pos, fromCoord));
                    }
                    if (finalCondition && finalCut.getCutLinkIndex() == linkIndex) {
                        toAdd = true;
                        double pos = finalCut.getCutPosition();
                        toCoord = VectOp.addVectors(VectOp.extPdt(pos, toCoord), VectOp.extPdt(1-pos, fromCoord));
                    }
                }
                if (toAdd) {
                    Coord vector = VectOp.addVectors(toCoord, VectOp.extPdt(-1, fromCoord));
                    links.get(segmentIndex).add(new LocalizedVector(fromCoord, vector));
                }
            }
        }
        return links;
    }


    /**
     * Return list of link ids without these removed because of Cuts
     * Each element of outer list (lists of Link ids) represents a segment
     * Each element of inner lists is a link id in the segment the link belongs to
     * @return list of lists of the Polyline's Link Ids
     */
    public ArrayList<ArrayList<Id<Link>>> getLinkIdsWithCuts() {
        ArrayList<ArrayList<Id<Link>>> links = new ArrayList<>();
        for (int segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {
            Segment segment = segments.get(segmentIndex);
            links.add(new ArrayList<>());
            for (int linkIndex = 0; linkIndex < segment.getLinks().size(); linkIndex++) {
                Link link = segment.getLinks().get(linkIndex);
                boolean initialCondition = segmentIndex==0 && initialCut.getIsLinkCut() && linkIndex < initialCut.getCutLinkIndex();
                boolean finalCondition = segmentIndex==segments.size()-1 && finalCut.getIsLinkCut() && linkIndex > finalCut.getCutLinkIndex();
                if (!initialCondition && !finalCondition) {
                    links.get(linkIndex).add(link.getId());
                }
            }
        }
        return links;
    }


    /**
     * @return sum of the lengths of all links removing cutted parts
     */
    public double getLengthWithCuts() {
        double totalLength = 0;
        ArrayList<ArrayList<LocalizedVector>> linksInSegments = getLinkLocalizedVectorsWithCuts();
        for (int segmentIndex = 0; segmentIndex < linksInSegments.size(); segmentIndex++) {
            ArrayList<LocalizedVector> segmentLinks = linksInSegments.get(segmentIndex);
            for (int linkIndex = 0; linkIndex < segmentLinks.size(); linkIndex++) {
                totalLength += VectOp.length(segmentLinks.get(linkIndex).getVectorCoord());
            }
        }
        return totalLength;
    }


    /**
     * @param index position in Polyline of the segment one wants the LocalizedVector of
     * @return LocalizedVector of desired segment removing cutted parts
     */
    public LocalizedVector getSegmentLocalizedVectorWithCut(int index) {
        if (index >= segments.size()) {
            return null;
        } else {
            Segment segment = segments.get(index);
            Coord origin = segment.getFromNode().getCoord();
            Coord endPoint = segment.getToNode().getCoord();
            if (index == 0) {
                origin = this.getFromCoordWithCut();
            } if (index == segments.size()-1) {
                endPoint = this.getToCoordWithCut();
            }
            return new LocalizedVector(origin, VectOp.addVectors(endPoint, VectOp.extPdt(-1, origin)));
        }
    }


    /**
     * Computes the distance between Polyline and point
     * @param point Coordinates of point one wants the distance with Polyline
     * @return Shortest distance between point and Polyline
     */
    public double distanceWithPoint(Coord point) {
        BidimensionalIndex cutLinkIndex = new BidimensionalIndex(0,0);
        double pointPos = closestPointInPolyline(point, cutLinkIndex);
        return VectOp.distance(point, polylinePointCoord(cutLinkIndex, pointPos));
    }


    /**
     * Looks for the closest point in Polyline from closeTerminalNode. If points are closer than tolerance, creates new Cut there
     * @param closeTerminalNode Node one wants to interpolate from
     * @param tolerance maximal distance between closeTerminalNode and closest point in Polyline
     * @param cutOnEnd decides if new Cut is initialCut (false) or finalCut (true)
     * @return New Polyline, with new Cut if it was possible
     */
    public Polyline interpolate(Coord closeTerminalNode, double tolerance, boolean cutOnEnd) {
        // Looking for the closest vertex in this
        BidimensionalIndex cutNodeIndex = closestVerticeInPolyline(closeTerminalNode);

        Coord cutCoord;
        if (cutNodeIndex.j == -2) {
            cutCoord = getFromCoordWithCut();
        } else if (cutNodeIndex.j == -1) {
            cutCoord = getToCoordWithCut();
        } else {
            cutCoord = segments.get(cutNodeIndex.i).getLinks().get(cutNodeIndex.j).getToNode().getCoord();
        }

        boolean nullLink = (cutOnEnd && cutNodeIndex.j == -2) || (!cutOnEnd && cutNodeIndex.j == -1);
        if (VectOp.distance(closeTerminalNode, cutCoord) < tolerance && !nullLink) {
            // If close enough, cut segment on link end & don't cut links
            ArrayList<Segment> segmentList = new ArrayList<>();
            if (cutOnEnd) {
                Cut newFinalCut;
                if (cutNodeIndex.j == -1) {
                    newFinalCut = new Cut(finalCut.getCutLinkIndex(), finalCut.getIsLinkCut(), finalCut.getCutPosition());
                    segmentList.addAll(segments);
                } else { // cannot be -2
                    newFinalCut = new Cut(cutNodeIndex.j, true, 1);
                    segmentList.addAll(segments.subList(0, 1+cutNodeIndex.i));
                }
                return new Polyline(segmentList, initialCut, newFinalCut);
            } else {
                Cut newInitialCut;
                if (cutNodeIndex.j == -2) {
                    newInitialCut = new Cut(initialCut.getCutLinkIndex(), initialCut.getIsLinkCut(), initialCut.getCutPosition());
                    segmentList.addAll(segments);
                } else { // cannot be -1
                    newInitialCut = new Cut(cutNodeIndex.j, true, 1);
                    segmentList.addAll(segments.subList(cutNodeIndex.i, segments.size()));
                }
                return new Polyline(segmentList, newInitialCut, finalCut);
            }
        }

        // cutting links in case link end not close enough
        double cutPos;
        BidimensionalIndex cutIndex = new BidimensionalIndex(0,0);
        if (cutNodeIndex.j < 0) {
            Link link;
            if (cutNodeIndex.j == -2) {
                cutIndex.i = 0;
                cutIndex.j = initialCut.getCutLinkIndex();
                if (!initialCut.getIsLinkCut()) {
                    cutIndex.j = 0;
                }
            } else { // cutLinkIndex == -1
                cutIndex.i = segments.size()-1;
                cutIndex.j = finalCut.getCutLinkIndex();
                if (!finalCut.getIsLinkCut()) {
                    cutIndex.j = segments.get(segments.size()-1).getLinks().size()-1;
                }
            }
            link = segments.get(cutIndex.i).getLinks().get(cutIndex.j);
            if (cutNodeIndex.j == -2) {
                cutPos = Math.max(initialCut.getCutPosition(), new LocalizedVector(link).closestPointInLocalizedVector(closeTerminalNode));
            } else {
                cutPos = Math.min(finalCut.getCutPosition(), new LocalizedVector(link).closestPointInLocalizedVector(closeTerminalNode));
            }
        } else {
            BidimensionalIndex cutIndex1 = new BidimensionalIndex(cutNodeIndex.i, cutNodeIndex.j);
            BidimensionalIndex cutIndex2 = new BidimensionalIndex(cutNodeIndex.i, cutNodeIndex.j+1);
            if (cutNodeIndex.j == segments.get(cutNodeIndex.i).getLinks().size() - 1) {
                cutIndex2.i += 1;
                cutIndex2.j = 0;
            }
            Link link1 = segments.get(cutIndex1.i).getLinks().get(cutIndex1.j);
            Link link2 = segments.get(cutIndex2.i).getLinks().get(cutIndex2.j);
            double cutPos1 = new LocalizedVector(link1).closestPointInLocalizedVector(closeTerminalNode);
            double cutPos2 = new LocalizedVector(link2).closestPointInLocalizedVector(closeTerminalNode);
            if (finalCut.isLinkCut && cutIndex2.i == segments.size()-1 && cutIndex2.j == finalCut.getCutLinkIndex() && cutPos2 > finalCut.getCutPosition()) {
                cutPos2 = finalCut.getCutPosition();
            }

            if (VectOp.distance(closeTerminalNode, polylinePointCoord(cutIndex1, cutPos1)) < VectOp.distance(closeTerminalNode, polylinePointCoord(cutIndex2, cutPos2))) {
                cutIndex = cutIndex1;
                cutPos = cutPos1;
            } else {
                cutIndex = cutIndex2;
                cutPos = cutPos2;
            }
        }

        if (VectOp.distance(polylinePointCoord(cutIndex, cutPos), closeTerminalNode) > tolerance) {
            return this;
        }


        if (cutOnEnd) {
            ArrayList<Segment> segmentList = new ArrayList<>(segments.subList(0, 1+cutIndex.i));
            Cut newFinalCut = new Cut(cutIndex.j, true, cutPos);
            if (cutIndex.j == segmentList.get(segmentList.size()-1).getLinks().size()-1 && cutPos == 1) {
                newFinalCut = new Cut(cutIndex.j, false, cutPos);
            }
            return new Polyline(segmentList, initialCut, newFinalCut);
        } else {
            ArrayList<Segment> segmentList = new ArrayList<>(segments.subList(cutIndex.i, segments.size()));
            Cut newInitialCut = new Cut(cutIndex.j, true, cutPos);
            if (cutIndex.j == 0 && cutPos == 0) {
                newInitialCut = new Cut(cutIndex.j, false, cutPos);
            }
            return new Polyline(segmentList, newInitialCut, finalCut);
        }
    }


    /**
     * Finds index for closest Polyline vertice (from/toNode of a link) from parameter point.
     * @param point Point one wants to find closet vertice from
     * @return closest vertice in Polyline
     * First index in return is segment placement in Polyline, second is Link placement in Segment.
     * Second index code : -2 for fromNodeWithCut ; -1 for toNodeWithCut.
     * Return points to a Link, vertice is link's toNode.
     */
    private BidimensionalIndex closestVerticeInPolyline(Coord point) {
        // Finding which node of the links of the segments in this Polyline is the closest to point
        int minLinkIndex = 0;
        if (initialCut.getIsLinkCut()) {
            minLinkIndex = initialCut.getCutLinkIndex();
        }
        int maxLinkIndex = segments.get(segments.size()-1).getLinks().size()-1;
        if (finalCut.getIsLinkCut()) {
            maxLinkIndex = finalCut.getCutLinkIndex();
        }
        BidimensionalIndex minimalLinkToNode = new BidimensionalIndex(0,-2);
        double minimalExistingNodeDistance = VectOp.distance(getFromCoordWithCut(), point);

        for (int segmentIndex = 0; segmentIndex < this.segments.size(); segmentIndex++) {
            Segment segment = this.segments.get(segmentIndex);
            int minIndex = 0;
            if (segmentIndex == 0) {
                minIndex = minLinkIndex;
            }
            int maxIndex = segment.getLinks().size()-1;
            if (segmentIndex == segments.size()-1) {
                maxIndex = maxLinkIndex;
            }
            for (int linkIndex = minIndex; linkIndex <= maxIndex; linkIndex++) {
                Coord toCoord = segment.getLinks().get(linkIndex).getToNode().getCoord();
                if (segmentIndex == segments.size()-1 && linkIndex == maxIndex && finalCut.getIsLinkCut()) { // final cut reached
                    toCoord = segment.segmentPointCoord(linkIndex, finalCut.getCutPosition());
                }
                double currentDistance = VectOp.distance(toCoord, point);
                if (currentDistance < minimalExistingNodeDistance) {
                    minimalLinkToNode.i = segmentIndex;
                    minimalLinkToNode.j = linkIndex;
                    minimalExistingNodeDistance = currentDistance;
                }
            }
        }

        if (minimalLinkToNode.i == segments.size()-1 && minimalLinkToNode.j == maxLinkIndex) {
            minimalLinkToNode.j = -1;
        }

        // Code: -2 for fromNodeWithCut ; -1 for toNodeWithCut
        return minimalLinkToNode;
    }


    /**
     * Finds closest Polyline point from parameter point
     * @param point coordinates of the point one wants to find closest Polyline point from (parameter point)
     * @param cutLinkIndex Can initially be any BidimentionalIndex, when function is executed, gets placement of the link the closest point belongs to
     * @return position of closest point in link indicated by cutLinkIndex (position: between 0 = fromNode and 1 = toNode)
     */
    public double closestPointInPolyline(Coord point, BidimensionalIndex cutLinkIndex) {
        // cutLinkIndex can be any BidimensionalIndex, It's getting changed anyway

        // Finding which node of the links of the segments in this Polyline is the closest to point
        int minLinkIndex = 0;
        if (initialCut.getIsLinkCut()) {
            minLinkIndex = initialCut.getCutLinkIndex();
        }
        int maxLinkIndex = segments.get(segments.size()-1).getLinks().size()-1;
        if (finalCut.getIsLinkCut()) {
            maxLinkIndex = finalCut.getCutLinkIndex();
        }
        BidimensionalIndex minimalLinkToNode = new BidimensionalIndex(0,-1);
        double minimalExistingNodeDistance = VectOp.distance(getFromCoordWithCut(), point);

        for (int segmentIndex = 0; segmentIndex < this.segments.size(); segmentIndex++) {
            Segment segment = this.segments.get(segmentIndex);
            int minIndex = 0;
            if (segmentIndex == 0) { minIndex = minLinkIndex; }
            int maxIndex = segment.getLinks().size()-1;
            if (segmentIndex == segments.size()-1) { maxIndex = maxLinkIndex; }
            for (int linkIndex = minIndex; linkIndex <= maxIndex; linkIndex++) {
                Coord toCoord = segment.getLinks().get(linkIndex).getToNode().getCoord();
                if (segmentIndex == segments.size()-1 && linkIndex == maxIndex && finalCut.getIsLinkCut()) { // final cut reached
                    toCoord = segment.segmentPointCoord(linkIndex, finalCut.getCutPosition());
                }
                double currentDistance = VectOp.distance(toCoord, point);
                if (currentDistance < minimalExistingNodeDistance) {
                    minimalLinkToNode.i = segmentIndex;
                    minimalLinkToNode.j = linkIndex;
                    minimalExistingNodeDistance = currentDistance;
                }
            }
        }

        // Analysing the respective closest point on the 2 links right before and right after the closest node
        double cutPos = 0;
        if (minimalLinkToNode.i == 0 && minimalLinkToNode.j == -1) {
            Link link = this.segments.get(minLinkIndex).getLinks().get(minLinkIndex);
            cutPos = (new LocalizedVector(link)).closestPointInLocalizedVector(point);
            if (initialCut.getIsLinkCut() && cutPos < initialCut.getCutPosition()) {
                cutPos = initialCut.getCutPosition();
            }
            cutLinkIndex.i = 0;
            cutLinkIndex.j = minLinkIndex;
        } else if (minimalLinkToNode.i == segments.size()-1 && minimalLinkToNode.j == maxLinkIndex) {
            Link link = this.segments.get(minimalLinkToNode.i).getLinks().get(minimalLinkToNode.j);
            cutPos = (new LocalizedVector(link)).closestPointInLocalizedVector(point);
            if (finalCut.getIsLinkCut() && cutPos > finalCut.getCutPosition()) {
                cutPos = finalCut.getCutPosition();
            }
            cutLinkIndex.i = minimalLinkToNode.i;
            cutLinkIndex.j = minimalLinkToNode.j;
        } else {
            BidimensionalIndex secondLink = new BidimensionalIndex(minimalLinkToNode.i, minimalLinkToNode.j+1);
            if (secondLink.j == segments.get(secondLink.i).getLinks().size()) {
                secondLink.i = secondLink.i + 1;
                secondLink.j = 0;
            }
            Link link1 = this.segments.get(minimalLinkToNode.i).getLinks().get(minimalLinkToNode.j);
            Link link2 = this.segments.get(secondLink.i).getLinks().get(secondLink.j);
            double pos1 = (new LocalizedVector(link1)).closestPointInLocalizedVector(point);
            double pos2 = (new LocalizedVector(link2)).closestPointInLocalizedVector(point);
            Coord point1 = VectOp.addVectors(VectOp.extPdt(1-pos1, link1.getFromNode().getCoord()), VectOp.extPdt(pos1, link1.getToNode().getCoord()));
            Coord point2 = VectOp.addVectors(VectOp.extPdt(1-pos2, link2.getFromNode().getCoord()), VectOp.extPdt(pos2, link2.getToNode().getCoord()));
            if (VectOp.distance(point, point2) < VectOp.distance(point, point1)) {
                cutLinkIndex.i = minimalLinkToNode.i;
                cutLinkIndex.j = minimalLinkToNode.j;
                cutPos = pos1;
            } else {
                cutLinkIndex.i = secondLink.i;
                cutLinkIndex.j = secondLink.j;
                cutPos = pos2;
            }
        }
        return cutPos;
    }


    /**
     * @param bidimensionalIndex placement of Segment and Link in Polyline
     * @param pos Position of point in Link (from 0 = fromNode to 1 = toNode)
     * @return Coordinates of the point indicated by bidimentionalIndex and pos
     */
    public Coord polylinePointCoord(BidimensionalIndex bidimensionalIndex, double pos) {
        Link link = this.segments.get(bidimensionalIndex.i).getLinks().get(bidimensionalIndex.j);
        return VectOp.addVectors(VectOp.extPdt(1-pos, link.getFromNode().getCoord()), VectOp.extPdt(pos, link.getToNode().getCoord()));
    }

    /**
     * Useless function
     * @param potentialCandidates
     * @param stitchOnToNode
     * @return
     */
    public HashSet<Polyline> stitch(HashSet<Segment> potentialCandidates, boolean stitchOnToNode) {
        HashSet<Polyline> finalSet = new HashSet<>();
        for (Segment segment : potentialCandidates) {
            if (!this.segments.contains(segment)) {
                if (stitchOnToNode) {
                    if (segment.getFromNode() == this.getToNodeWithoutCut()) {
                        ArrayList<Segment> segmentList = new ArrayList<>();
                        segmentList.addAll(this.segments);
                        segmentList.add(segment);
                        finalSet.add(new Polyline(segmentList, getInitialCut(), new Cut(segments.get(segments.size()-1).getLinks().size()-1, false, 1)));
                    }
                } else {
                    if (segment.getToNode() == this.getFromNodeWithoutCut()) {
                        ArrayList<Segment> segmentList = new ArrayList<>();
                        segmentList.add(segment);
                        segmentList.addAll(this.segments);
                        finalSet.add(new Polyline(segmentList, new Cut(0, false, 0), getFinalCut()));
                    }
                }
            }
        }
        return finalSet;
    }


    /**
     * Removes False cuts (ex. isLinkCut = true but cutPosition is 0 or 1 => set isLinkCut to false)
     */
    protected void correctFalseCuts() {
        if (initialCut.getIsLinkCut() && initialCut.getCutLinkIndex() == 0 && initialCut.getCutPosition() == 0) {
            initialCut.isLinkCut = false;
        }
        if (finalCut.getIsLinkCut() && finalCut.getCutLinkIndex() == segments.get(segments.size()-1).getLinks().size()-1 && finalCut.getCutPosition() == 1) {
            finalCut.isLinkCut = false;
        }
    }


    /**
     * Checks if other Polyline is equal to this (same segments and same cuts)
     * @param pl other Polyline to check
     * @return true if equal, false otherwies
     */
    public boolean equals(Polyline pl) {
        this.correctFalseCuts();
        pl.correctFalseCuts();
        boolean equalInitCut = (!initialCut.getIsLinkCut() && !pl.initialCut.getIsLinkCut()) || initialCut.equals(pl.initialCut);
        boolean equalFinalCut = (!finalCut.getIsLinkCut() && !pl.finalCut.getIsLinkCut()) || finalCut.equals(pl.finalCut);
        return segments.equals(pl.getSegments()) && equalInitCut && equalFinalCut;
    }


    /**
     * Cut in a Polyline if Polyline have been interpolated in case one of its endpoints wasn't close enough to refSegment's endpoints
     * Contains information about Cut's location
     */
    class Cut {
        private int cutLinkIndex; // Index of the cutLink on the segment links list
        private boolean isLinkCut;
        private double cutPosition; // between 0 and 1 relative to the length of the link


        /**
         * @param cutLinkIndex represents the Link this Cut is in (index being link placement in Segment)
         * @param isLinkCut true if link is cut, false otherwise
         * @param cutPosition position of Cut in Link (from 0 = Link's fromNode to 1 = Link's toNode)
         */
        Cut(int cutLinkIndex, boolean isLinkCut, double cutPosition) {
            this.cutLinkIndex = cutLinkIndex;
            this.isLinkCut = isLinkCut;
            this.cutPosition = cutPosition;
        }


        /**
         * Checks if other cut is equal to this (i.e. with same isLinkCut status, cutLinkIndex and cutPosition)
         * @param cut other Cut
         * @return true it equal, false otherwise
         */
        public boolean equals(Cut cut) {
            return (isLinkCut==cut.getIsLinkCut() && cutLinkIndex==cut.getCutLinkIndex() && Math.abs(cutPosition-cut.getCutPosition())<0.0001);
        }


        public int getCutLinkIndex() {
            return this.cutLinkIndex;
        }


        public boolean getIsLinkCut() {
            return this.isLinkCut;
        }


        public double getCutPosition() {
            return this.cutPosition;
        }
    }
}
