package main;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.core.network.NetworkUtils;

import java.util.*;


/**
 * Contains the information and functions to conflate the network
 * Including two ConflationPreprocessedNetwork's
 */
public class NetworkConflator {

    private ConflationPreprocessedNetwork refNet;
    private ConflationPreprocessedNetwork targetNet;
    private double nodeTolerance;

    private final static Logger LOG = LogManager.getLogger(NetworkConflator.class);
    // TODO make all logger messages

    // Default values
    private double refToTargetAngleTolerance = Math.PI/4;
    private double angleToleranceCos = Math.cos(refToTargetAngleTolerance);
    public double bufferMinimalOverlap = 0.2; // Must be >= 5% to avoid errors

    /**
     * Constructor for NetworkConflator, saves the parameters as NetworkConflator object attributes
     *
     * @param refPath                           path to the reference network (refNet)
     * @param refNodeTolerance                  tolerance for node location in refNet
     * @param refBufferTolerance                tolerance for segment location in refNet
     * @param allRefNodesTerminal               if true, forces nodes to be terminal and all segments to be only one link in reference preprocessed network
     * @param targetPath                        path to the target network (targetNet)
     * @param targetNodeTolerance               tolerance for node location in targetNet
     * @param targetBufferTolerance             tolerance for segment location in targetNet
     * @param allTargetNodesTerminal            if true, forces nodes to be terminal and all segments to be only one link in target preprocessed network
     * @param rTreeSquareDimension              rTree associates neighbouring segments by making a grid. Segments crossing same squarein grid placed in same rTree branch. One segment can be in several branches. rTreeSquareDimension must be > refBufferTolerance + targetBufferTolerance
     * @param modesToKeep                       keep in Networks only the links allowing one mode in modesToKeep
     * @param savePreprocessedNetworks          after making segments out of several links (between terminal nodes) save the segments as links in a new .xml file
     * @param simplifiedRefNetworkSavingPath    paths where simplified refNet is saved
     * @param simplifiedTargetNetworkSavingPath paths where simplified targetNet is saved
     */
    public NetworkConflator(String refPath, double refNodeTolerance, double refBufferTolerance, boolean allRefNodesTerminal,
                            String targetPath, double targetNodeTolerance, double targetBufferTolerance, boolean allTargetNodesTerminal,
                            double rTreeSquareDimension, HashSet<String> modesToKeep, boolean savePreprocessedNetworks,
                            String simplifiedRefNetworkSavingPath, String simplifiedTargetNetworkSavingPath) {

        if (rTreeSquareDimension <= refBufferTolerance + targetBufferTolerance) {
            rTreeSquareDimension = refBufferTolerance+targetBufferTolerance+1;
            LOG.warn("R-Tree square dimension too low, changed to "+rTreeSquareDimension);
        }

        Network refNet = NetworkUtils.readNetwork(refPath);
        Network targetNet = NetworkUtils.readNetwork(targetPath);

        this.refNet = new ConflationPreprocessedNetwork(refNet, refBufferTolerance, refNodeTolerance, rTreeSquareDimension);
        this.targetNet = new ConflationPreprocessedNetwork(targetNet, targetBufferTolerance, targetNodeTolerance, rTreeSquareDimension);
        this.nodeTolerance = this.refNet.getNodeTolerance() + this.targetNet.getNodeTolerance();

        this.refNet.preprocessNetwork(Math.PI/6, modesToKeep, false);
        this.targetNet.preprocessNetwork(Math.PI/6, modesToKeep, false);

        if (savePreprocessedNetworks) {
            this.refNet.saveSimplifiedNetwork(simplifiedRefNetworkSavingPath);
            this.targetNet.saveSimplifiedNetwork(simplifiedTargetNetworkSavingPath);
        }
    }


    public ConflationPreprocessedNetwork getRefNet() {
        return refNet;
    }

    public ConflationPreprocessedNetwork getTargetNet() {
        return targetNet;
    }


    /**
     * Sets max angle for segment to be kept as segment candidate
     * @param refToTargetAngleTolerance angle tolerance (radians)
     */
    public void setRefToTargetAngleTolerance(double refToTargetAngleTolerance) {
        this.refToTargetAngleTolerance = refToTargetAngleTolerance;
        this.angleToleranceCos = Math.cos(refToTargetAngleTolerance);
    }


    /**
     * Sets minimal overlap threshold for segment to be kept as segment candidate
     * @param bufferMinimalOverlap minimal value for overlapArea / min(refSegment buffer, targetSegment buffer) to keep segment as candidate
     */
    public void setBufferOverlapPruneThreshold(double bufferMinimalOverlap) {
        this.bufferMinimalOverlap = bufferMinimalOverlap;
    }


    /**
     * Highest level main function of the class: builds targetNet ScoredPolyline good candidates for each refNet segment
     * @return Map of refSegment id <-> HashSet of targetNet ScoredPolyline good candidates for refSegment
     */
    public HashMap<Long, HashSet<ScoredPolyline>> populateForNetwork() {
        // Algo 1, p 49
        HashMap<Long, HashSet<ScoredPolyline>> goodPotentialMatches = new HashMap<>();
        int seenSegments = 0;
        int totalSegments = refNet.getSegments().size();
        for (Segment refSegment : new HashSet<>(refNet.getSegments().values())) {
            seenSegments += 1;
            populateOneSegment(refSegment, goodPotentialMatches);
            if ((int) (100.0*seenSegments/totalSegments) != (int) (100.0*(seenSegments-1)/totalSegments)) {
                LOG.info((int) (100*seenSegments/totalSegments) + "% segments populated");
            }
        }
        return goodPotentialMatches;
    }


    /**
     * Used in populateForNetwork : Constructs Polyline good candidates and scores them for one refSegment. Can cut refSegment in 2 or 3 shorter segments if no good candidate is found
     * @param refSegment segment from refNet
     * @param goodPotentialMatches initially empty Set of good candidates for refSegment, can not be empty if function is calling itself
     */
    private void populateOneSegment(Segment refSegment, HashMap<Long, HashSet<ScoredPolyline>> goodPotentialMatches) {
        int refNOfLinks = refSegment.getLinks().size();
        double refSegStraightLineLength = refSegment.getStraightLineLength();
        HashSet<Segment> potentialCandidateMatches = findSegmentCandidateMatches(refSegment);

        HashSet<Polyline> candidates = buildPolylineCandidateMatches(refSegment, potentialCandidateMatches);
        HashSet<ScoredPolyline> candidateMatches = new HashSet<>();

        for (Polyline candidate : candidates) {
            candidateMatches.add(new ScoredPolyline(candidate, computeScore(refSegment, candidate)));
        }

        // finding if goodPotentialMatches has actual good candidates
        boolean goodCandidateExists = false;
        for (ScoredPolyline candidate : candidateMatches) {
            Coord fromCoord = candidate.getPolyline().getFromCoordWithCut();
            Coord toCoord = candidate.getPolyline().getToCoordWithCut();
            double fromNodesDistance = VectOp.distance(fromCoord, refSegment.getFromNode().getCoord());
            double toNodesDistance = VectOp.distance(toCoord, refSegment.getToNode().getCoord());
            if (fromNodesDistance < nodeTolerance && toNodesDistance < nodeTolerance) {
                goodCandidateExists = true;
                break;
            }
        }
        if (goodCandidateExists) {
            // Deleting all partial matches
            HashSet<ScoredPolyline> candidateMatchesCopy = new HashSet<>(candidateMatches);
            for (ScoredPolyline candidate : candidateMatchesCopy) {
                double fromNodesDistance = VectOp.distance(candidate.getPolyline().getFromCoordWithCut(), refSegment.getFromNode().getCoord());
                double toNodesDistance = VectOp.distance(candidate.getPolyline().getToCoordWithCut(), refSegment.getToNode().getCoord());
                if (fromNodesDistance > nodeTolerance || toNodesDistance > nodeTolerance) {
                    candidateMatches.remove(candidate);
                }
            }
            goodPotentialMatches.put(refSegment.getId(), candidateMatches);
        } else {
            // Case of partial matches :
            // Finding the best scoring partial match where both endpoints are close to any point in refSegment
            // Interpolating refSegment for the segments to correspond to its best scoring partial match
            ArrayList<ScoredPolyline> sortedCandidateMatches = new ArrayList<>(candidateMatches);
            sortedCandidateMatches.sort(Comparator.comparingDouble(sp -> sp.score));

            ArrayList<Long> newSegments = new ArrayList<>();
            for (int i = sortedCandidateMatches.size()-1; i > 0; i--) {
                Polyline candidate = sortedCandidateMatches.get(i).getPolyline();
                Coord candidateFromCoord = candidate.getFromCoordWithCut();
                Coord candidateToCoord = candidate.getToCoordWithCut();
                Coord segmentFromCoord = refSegment.getFromNode().getCoord();
                Coord segmentToCoord = refSegment.getToNode().getCoord();
                if (VectOp.distance(candidateFromCoord, segmentFromCoord) <= nodeTolerance) {    // From-nodes are close
                    ArrayList<Integer> linkIndex = new ArrayList<>();
                    double pos = refSegment.closestPointInSegment(candidateToCoord, linkIndex);
                    int index = linkIndex.get(0);
                    boolean nullLink = index + pos < 0.0001 || index + pos > refNOfLinks-0.0001;
                    if (VectOp.distance(candidateToCoord, refSegment.segmentPointCoord(index, pos)) <= nodeTolerance && !nullLink) {
                        newSegments = refNet.cutSegment(refSegment.getId(), index, pos);
                        break;
                    }
                } else if (VectOp.distance(candidateToCoord, segmentToCoord) <= nodeTolerance) { // To-nodes are close
                    ArrayList<Integer> linkIndex = new ArrayList<>();
                    double pos = refSegment.closestPointInSegment(candidateFromCoord, linkIndex);
                    int index = linkIndex.get(0);
                    boolean nullLink = index + pos < 0.0001 || index + pos > refNOfLinks-0.0001;
                    if (VectOp.distance(candidateFromCoord, refSegment.segmentPointCoord(index, pos)) <= nodeTolerance && !nullLink) {
                        newSegments = refNet.cutSegment(refSegment.getId(), index, pos);
                        break;
                    }
                } else {                                                                        // No node is close
                    ArrayList<Integer> linkIndex1 = new ArrayList<>();
                    double pos1 = refSegment.closestPointInSegment(candidateFromCoord, linkIndex1);
                    int index1 = linkIndex1.get(0);
                    Coord point1 = refSegment.segmentPointCoord(index1, pos1);
                    ArrayList<Integer> linkIndex2 = new ArrayList<>();
                    double pos2 = refSegment.closestPointInSegment(candidateToCoord, linkIndex2);
                    int index2 = linkIndex2.get(0);
                    Coord point2 = refSegment.segmentPointCoord(index2, pos2);
                    boolean nullLink = index1 + pos1 < 0.0001 || index2 + pos2 > refNOfLinks-0.0001 || (index2+pos2) - (index1+pos1) < 0.0002;
                    if (VectOp.distance(candidateFromCoord, point1) <= nodeTolerance && VectOp.distance(candidateToCoord, point2) <= nodeTolerance && !nullLink) {
                        if (pos1 > 0.9999) {
                            index1 += 1;
                            pos1 = 0;
                        }
                        newSegments = refNet.cutSegment(refSegment.getId(), index1, pos1);
                        if (index2 == index1) {
                            pos2 = pos2 - pos1;
                        }
                        ArrayList<Long> otherNewSegments = refNet.cutSegment(newSegments.get(1), index2 - index1, pos2);
                        newSegments.remove(1);
                        newSegments.addAll(otherNewSegments);
                        break;
                    }
                }
            }
            // If no potential match found don't do anything

            // looking for good candidates for each new interpolated segment (recursive)

            for (Long segId : newSegments) {
                Segment newSegment = refNet.getSegments().get(segId);
                populateOneSegment(newSegment, goodPotentialMatches);
            }
        }
    }


    /**
     * Builds as many good candidates as possible (but still keeps Polylines that cannot be interpolated into good candidates)
     * @param refSegment segment from refNet
     * @param segmentCandidates Set of targetSegments kept as segment candidates for refSegment
     * @return Set of all possible Polylines built from targetSegments in potentialMatches
     */
    public HashSet<Polyline> buildPolylineCandidateMatches(Segment refSegment, HashSet<Segment> segmentCandidates) {
        HashMap<Segment, HashSet<Polyline>> polylinesFromSegments = new HashMap<>();

        // Filling startingSegments with segments that don't have next in PotentialMatches
        buildPolylines(segmentCandidates, polylinesFromSegments);

        // Interpolating built polylines where from/to-nodes don't correspond with refSegment's
        HashSet<Polyline> finalSet = new HashSet<>();
        for (HashSet<Polyline> plSet : polylinesFromSegments.values()) {
            for (Polyline pl : plSet) {
                Polyline interpolatedPl = new Polyline(pl.getSegments(), pl.getInitialCut(), pl.getFinalCut());
                double fromNodeDistance = VectOp.distance(interpolatedPl.getFromCoordWithCut(), refSegment.getFromNode().getCoord());
                double toNodeDistance = VectOp.distance(interpolatedPl.getToCoordWithCut(), refSegment.getToNode().getCoord());
                if (toNodeDistance > nodeTolerance) {
                    interpolatedPl = interpolatedPl.interpolate(refSegment.getToNode().getCoord(), nodeTolerance, true);
                }
                if (fromNodeDistance > nodeTolerance) {
                    interpolatedPl = interpolatedPl.interpolate(refSegment.getFromNode().getCoord(), nodeTolerance, false);
                }

                interpolatedPl.simplifyCuts();
                if (interpolatedPl.getSegments().size() > 0) {
                    boolean alreadyPresent = false;
                    for (Polyline candidate : finalSet) {
                        if (candidate.equals(interpolatedPl)) {
                            alreadyPresent = true;
                            break;
                        }
                    }
                    if (!alreadyPresent) {
                        finalSet.add(interpolatedPl);
                    }
                }
            }
        }
        return finalSet;
    }


    /**
     * Used in buildPolylineCandidateMatches : builds all possible candidates from Segments in potentialCandidates
     * @param potentialCandidates Set of Segments from which function must build Polylines
     * @param polylinesFromSegments Map associating First segment in Polyline <-> Set of Polylines. Initially empty except when function calls itself. At the end, contains all built Polylines
     */
    private void buildPolylines(HashSet<Segment> potentialCandidates, HashMap<Segment, HashSet<Polyline>> polylinesFromSegments) {
        for (Segment startingSeg : potentialCandidates) {
            if (!polylinesFromSegments.containsKey(startingSeg)) {
                boolean hasUnseenNext = false;
                for (Segment segment : potentialCandidates) {
                    if (startingSeg.getToNode() == segment.getFromNode() && !polylinesFromSegments.containsKey(segment)) {
                        hasUnseenNext = true;
                        break;
                    }
                }
                if (!hasUnseenNext) {
                    // adding polyline composed of only startingSeg
                    ArrayList<Segment> segmentList = new ArrayList<>();
                    segmentList.add(startingSeg);
                    HashSet<Polyline> polylines = new HashSet<>();
                    polylines.add(new Polyline(segmentList));
                    polylinesFromSegments.put(startingSeg, polylines);

                    // adding Polylines starting from startingSeg's successors: startingSeg doesn't have unseen next
                    // => all possible Polylines starting from its successors have already been built
                    for (Segment segment : polylinesFromSegments.keySet()) {
                        if (startingSeg.getToNode() == segment.getFromNode()) {
                            for (Polyline polyline : polylinesFromSegments.get(segment)) {
                                ArrayList<Segment> newSegmentList = new ArrayList<>();
                                newSegmentList.add(startingSeg);
                                newSegmentList.addAll(polyline.getSegments());
                                polylinesFromSegments.get(startingSeg).add(new Polyline(newSegmentList));
                            }
                        }
                    }
                    buildPolylines(potentialCandidates, polylinesFromSegments);
                }
            }
        }
    }


    /**
     * Prints initial cut, link ids and final cut for a Polyline
     * @param candidate Polyline to print
     */
    public void printLinks(Polyline candidate) {
        ArrayList<ArrayList<Id<Link>>> linkIds = candidate.getLinkIdsWithCuts();
        if (candidate.getInitialCut().getIsLinkCut()) {
            System.out.print("Init cut: "+candidate.getInitialCut().getCutPosition()+"; ");
        }
        for (ArrayList<Id<Link>> linksInSegment : linkIds) {
            for (Id<Link> linkId : linksInSegment) {
                targetNet.printLink(linkId);
            }
        }
        if (candidate.getFinalCut().getIsLinkCut()) {
            System.out.print("Fin. cut: "+candidate.getFinalCut().getCutPosition());
        }
        System.out.println("");
    }


    /**
     * Used in populateOneSegment : Computes similarity score for a Polyline candidate to represent refSegment
     * @param refSegment segment from refNet
     * @param candidate Polyline of which function calculates similarity score
     * @return candidate score
     */
    private double computeScore(Segment refSegment, Polyline candidate) {
        double ovArea = 0;
        double tarArea = 0;
        /* Assuming the angles between links are not fairly small
        we consider the buffer of the polyline as the union of the buffers of links
        (without lengthening them at their ends) */
        ArrayList<ArrayList<LocalizedVector>> linksInSegments = candidate.getLinkLocalizedVectorsWithCuts();

        for (int segIndex = 0; segIndex < linksInSegments.size(); segIndex++) {
            for (int linkIndex = 0; linkIndex < linksInSegments.get(segIndex).size(); linkIndex++) {
                boolean prolongTarFrom = (segIndex == 0 && linkIndex == 0);
                boolean prolongTarTo = (segIndex == linksInSegments.size()-1 && linkIndex == linksInSegments.get(segIndex).size() - 1);
                LocalizedVector linkLocalizedVector = linksInSegments.get(segIndex).get(linkIndex);
                tarArea += 2*targetNet.getBufferTolerance() * VectOp.length(linkLocalizedVector.getVectorCoord());
                ovArea += overlapArea(refSegment.getLocalizedVector(), linkLocalizedVector, prolongTarFrom, prolongTarTo);
            }
        }
        tarArea += 4*Math.pow(targetNet.getBufferTolerance(), 2); // add the extremities that were not yet taken into account
        double refArea = 2*refNet.getBufferTolerance() * (refSegment.getStraightLineLength() + 2*refNet.getBufferTolerance());
        return ovArea / (refArea + tarArea - ovArea);         // inter(ref, tar) / union(ref, tar)
    }


    /**
     * Used in populateOneSegment : Looks in targetNet for nearby Segments to be kept as candidates for refSegment
     * @param refSegment segment from refNet
     * @return Set of candidate Segments
     */
    public HashSet<Segment> findSegmentCandidateMatches(Segment refSegment) {
        HashSet<Long> nearbySegments = new HashSet<>();
        HashMap<Integer, HashSet<Integer>> viewedRTreeBranches = new HashMap<>();
        int n = refSegment.getRTreeBranchesSet().size();
        int m = 0;
        for (BidimensionalIndex k : refSegment.getRTreeBranchesSet()) {
            m++;
            for (int i = k.i - 1; i<k.i + 2; i++) {
                for (int j = k.j - 1; j<k.j + 2; j++) {
                    if (targetNet.getRTree().containsKey(i) && targetNet.getRTree().get(i).containsKey(j)) {
                        if (!viewedRTreeBranches.containsKey(i)) {
                            viewedRTreeBranches.put(i, new HashSet<>());
                        }
                        if (!viewedRTreeBranches.get(i).contains(j)) {
                            nearbySegments.addAll(targetNet.getRTree().get(i).get(j));
                            viewedRTreeBranches.get(i).add(j);
                        }
                    }
                }
            }
        }
        HashSet<Segment> potentialMatches = new HashSet<>();
        for (Long targetSegmentId : nearbySegments) {
            Segment targetSegment = targetNet.getSegments().get(targetSegmentId);
            boolean isPotentialMatch = true;
            if (VectOp.cosine(refSegment.getCoord(), targetSegment.getCoord()) < angleToleranceCos) {
                isPotentialMatch = false;
            } else {
                double refBufferArea = 2*refNet.getBufferTolerance() * (refSegment.getStraightLineLength() + 2*refNet.getBufferTolerance());
                double targetBufferArea = 2*targetNet.getBufferTolerance() * targetSegment.getStraightLineLength() + 2*targetNet.getBufferTolerance();
                if (overlapArea(refSegment.getLocalizedVector(), targetSegment.getLocalizedVector(), true, true) / Math.min(refBufferArea, targetBufferArea) < bufferMinimalOverlap) {
                    isPotentialMatch = false;
                }
            }
            if (isPotentialMatch) {
                potentialMatches.add(targetSegment);
            }
        }
        return potentialMatches;
    }


    private double getLinkLength(Link link) {
        Coord fromCoord = link.getFromNode().getCoord();
        Coord toCoord = link.getToNode().getCoord();
        return VectOp.length(VectOp.addVectors(fromCoord, VectOp.extPdt(-1, toCoord)));
    }


    /**
     * Computes the intersection area between the buffer zones of two segments (i.e. rectangle around a Segment, sides of said rectangle distant of bufferTolerance from Segment)
     * @param refSeg segment from refNet
     * @param targetSeg segment from targetNet
     * @param prolongTarFrom short side of rectangle passes at a distance from fromNode (true) or through it (false)
     * @param prolongTarTo short side of rectangle passes at a distance from toNode (true) or through it (false)
     * @return area of intersection of the two buffers
     */
    public double overlapArea(LocalizedVector refSeg, LocalizedVector targetSeg, boolean prolongTarFrom, boolean prolongTarTo) {
        if ((refSeg.getVectorCoord().getX() == 0 && refSeg.getVectorCoord().getX() == 0) || (targetSeg.getVectorCoord().getX() == 0 && targetSeg.getVectorCoord().getX() == 0)) {
            return 0;
        }

        double refTolerance = refNet.getBufferTolerance();
        double targetTolerance = targetNet.getBufferTolerance();

        // Construction of the buffer zone around seg1 : first left border, then right
        double dot = VectOp.dot(refSeg.getVectorCoord(), targetSeg.getVectorCoord());
        double vectorPdt = VectOp.vectorPdt(refSeg.getVectorCoord(), targetSeg.getVectorCoord());
        double inclination = dot*vectorPdt;
        int inclinationSign = sign(inclination);

        // Depending on the directions of seg1, seg2 : 1st buffer border of seg2 is the one with minimal x when expressed in orthonormal base from seg1
        int leftRightOrder = sign(vectorPdt);
        double targetFromPlonong = 0;
        double targetToPlonong = 0;
        if (prolongTarFrom) {
            targetFromPlonong = targetTolerance; // targetNet.getBufferTolerance();
        }
        if (prolongTarTo) {
            targetToPlonong = targetTolerance; // targetNet.getBufferTolerance();
        }
        ArrayList<LocalizedVector> buffer = new ArrayList<>();
        buffer.add(targetSeg.getBufferBorder(leftRightOrder*targetTolerance, targetFromPlonong, targetToPlonong));
        buffer.add(targetSeg.getBufferBorder(-leftRightOrder*targetTolerance, targetFromPlonong, targetToPlonong));

        // Expressing the departure points as translations from seg1.fromNode
        // Then expressing all coordinates in refSeg base
        for (int i = 0; i<2; i++) {
            buffer.set(i, buffer.get(i).expressInOrthoNBase(refSeg.getFromCoord(), refSeg.getVectorCoord()));
        }

        if (dot == 0 || vectorPdt == 0) {
            double xMin;
            double xMax;
            double yMin;
            double yMax;
            if (dot == 0) { // segments are perpendicular
                xMin = Math.max(-refTolerance, buffer.get(0).getFromCoord().getX());
                xMax = Math.min(VectOp.length(refSeg.getVectorCoord())+refTolerance, buffer.get(1).getFromCoord().getX());
                yMin = Math.max(-refTolerance, Math.min(buffer.get(0).getFromCoord().getY(), buffer.get(0).getToCoord().getY()));
                yMax = Math.min(refTolerance, Math.max(buffer.get(0).getFromCoord().getY(), buffer.get(0).getToCoord().getY()));
            } else { // segments are parallel
                xMin = Math.max(-refTolerance, Math.min(buffer.get(0).getFromCoord().getX(), buffer.get(0).getToCoord().getX()));
                xMax = Math.min(VectOp.length(refSeg.getVectorCoord())+refTolerance, Math.max(buffer.get(0).getFromCoord().getX(), buffer.get(0).getToCoord().getX()));
                yMin = Math.max(-refTolerance, Math.min(buffer.get(0).getFromCoord().getY(), buffer.get(1).getFromCoord().getY()));
                yMax = Math.min(refTolerance, Math.max(buffer.get(0).getFromCoord().getY(), buffer.get(1).getFromCoord().getY()));
            }
            return Math.max(0, xMax-xMin)*Math.max(0, yMax-yMin);
        }

        /// Starting by drawing a parallelogram with the lengths of both rectangles
        /// First two bottom points: y = -refBufferTolerance, left->right , then two top points: y = +refBufferTolerance, right->left
        ArrayList<Coord> parallelogram = buildParallelogram(buffer);

        /// Removing what is cut by the width of rect1
        ArrayList<Integer> cutCase = new ArrayList<>(); // first elt: left cut, last: right cut
        ArrayList<Coord> leftCut = new ArrayList<>();
        ArrayList<Coord> rightCut = new ArrayList<>();

        double xMinL = Math.min(parallelogram.get(0).getX(), parallelogram.get(3).getX());
        double xMaxL = Math.max(parallelogram.get(0).getX(), parallelogram.get(3).getX());
        double xMinR = Math.min(parallelogram.get(1).getX(), parallelogram.get(2).getX());
        double xMaxR = Math.max(parallelogram.get(1).getX(), parallelogram.get(2).getX());
        double offset = xMaxL - xMinL;

        // Left cut : x = -refBufferTolerance
        double lcut = -refTolerance;
        if (xMaxR < lcut) {
            return 0;
        } else {
            // Left cut vs left parallelogram side
            if (lcut <= xMinL) {
                cutCase.add(0); // parallelogram remains intact
            } else if (lcut < xMaxL) {
                leftCut = newVerticalCut(lcut, -refTolerance, refTolerance * (2*(lcut-xMinL)/offset - 1));
                if (inclination < 0) {
                    cutVerticalSymmetry(leftCut);
                }
                cutCase.add(1); // rectangle 1 width cuts parallelogram tilted side
            } else { // xMaxL <= lcut
                leftCut = newVerticalCut(lcut, -refTolerance, refTolerance);
                cutCase.add(3); // rectangle 1 width remains intact: cuts through whole parallelogram
            }

            // Left cut vs right parallelogram side
            if (xMinR < lcut) {
                int i = Math.round(-sign(inclination)/2+0.5f); // 1 if negative, 0 if positive
                leftCut.set(i, new Coord(lcut, refTolerance*inclinationSign*(2*(-refTolerance-xMinR)/offset - 1)));
                cutCase.set(0, cutCase.get(0)+1);
            }
        }

        // Right cut : x = length + tolerance
        double rcut = VectOp.length(refSeg.getVectorCoord()) + refTolerance;
        if (rcut < xMinL) {
            return 0;
        } else {
            // Right cut vs right parallelogram side
            if (xMaxR <= rcut) {
                cutCase.add(0); // parallelogram remains intact
            } else if (xMinR <= rcut) {
                rightCut = newVerticalCut(rcut,refTolerance * (2*(rcut-xMinR)/offset - 1), refTolerance);
                if (inclination < 0) {
                    cutVerticalSymmetry(rightCut);
                }
                cutCase.add(1); // rectangle 1 width cuts parallelogram tilted side
            } else { // rcut < xMinR
                rightCut = newVerticalCut(rcut, -refTolerance, refTolerance);
                cutCase.add(3); // rectangle 1 width remains intact: cuts through whole parallelogram
            }

            // Right cut vs left parallelogram side
            if (rcut < xMaxL) {
                int i = Math.round((inclinationSign+1)/2);
                rightCut.set(i, new Coord(rcut, refTolerance * inclinationSign * (2*(rcut-xMinL)/offset - 1)));
                cutCase.set(1, cutCase.get(1)+1);
            }
        }

        /// Saving the polygon calculated so far unto a graph structure
        Polygon polygon = new Polygon();

        if (cutCase.get(0) != 0) {
            polygon.addVertice(10, leftCut.get(0));
            polygon.addVertice(11, leftCut.get(1));
            polygon.addSide(10, 11);
        }
        if (cutCase.get(1) != 0) {
            polygon.addVertice(20, rightCut.get(0));
            polygon.addVertice(21, rightCut.get(1));
            polygon.addSide(20, 21);
        }

        for (int i=0; i<4; i++) {
            if (lcut < parallelogram.get(i).getX() && parallelogram.get(i).getX() < rcut) {
                polygon.addVertice(i, parallelogram.get(i));
            }
        }

        boolean centralVerticeAdded = false;
        for (int i=0; i<4; i++) { // Adding central parallelogram vertices and sides
            if (polygon.containsVertice(i)) {
                centralVerticeAdded = true;
                int j = i+1;
                while (!polygon.containsVertice(j%4)) { j = j+1; }
                if (j == i+1) {
                    polygon.addSide(i,j%4);
                } else {
                    boolean isCutLeft = false;
                    boolean isCutRight = false;
                    for (int k=i+1; k<j; k++) {
                        if (parallelogram.get(k%4).getX() < lcut) { isCutLeft = true; }
                        else if (rcut < parallelogram.get(k%4).getX()) { isCutRight = true; }
                    }
                    if (isCutRight && !isCutLeft) {
                        polygon.addSide(i,20);
                        polygon.addSide(21,j%4);
                    } else if (isCutLeft && !isCutRight) {
                        polygon.addSide(i,11);
                        polygon.addSide(10,j%4);
                    } else {
                        if (parallelogram.get(i).getY() < 0) {
                            polygon.addSide(i,20);
                            polygon.addSide(21,11);
                            polygon.addSide(10,j%4);
                        } else {
                            polygon.addSide(i,11);
                            polygon.addSide(10,20);
                            polygon.addSide(21,j%4);
                        }
                    }
                }
            }
        }
        if (!centralVerticeAdded) {
            polygon.addSide(20, 10);
            polygon.addSide(11, 21);
        }

        /// Expressing in seg2 base (as everything else is in seg1 base, first express seg2 in seg1 base then convert all to seg2 base)
        LocalizedVector targetSegInRefSegBase = targetSeg.expressInOrthoNBase(refSeg.getFromCoord(), refSeg.getVectorCoord());
        Coord substractedVector = VectOp.extPdt(-1, targetSegInRefSegBase.getFromCoord());
        for (int i : polygon.getCoordinatesMap().keySet()) {
            polygon.replaceVerticeCoord(i, VectOp.expressInOrthoNBase(VectOp.addVectors(polygon.getVerticeCoord(i), substractedVector), targetSegInRefSegBase.getVectorCoord()));
        }

        /// Saving a "width profile" (h = f(x)) in order to integrate between lcut and rcut
        lcut = -targetFromPlonong;
        rcut = VectOp.length(targetSeg.getVectorCoord()) + targetToPlonong;

        ArrayList<Double> xList = new ArrayList<>();
        ArrayList<Double> hList = polygon.getWidthProfile(xList);

        // Computing the final area accounting for left and right cuts on seg2
        int i0 = 0;
        while (i0<xList.size() && xList.get(i0) < lcut) {
            i0++;
        } // i0 is the first index i with xList.get(i) >= lcut
        int iF = i0;
        while (iF<xList.size() && xList.get(iF) <= rcut) {
            iF++;
        } // iF is the first index i with xList.get(i) > rcut
        ArrayList<Double> shortXList = new ArrayList<>();
        ArrayList<Double> shortHList = new ArrayList<>();

        // First value
        if (i0 == xList.size() || iF == 0) {
            return 0;
        } else if (i0 > 0) {
            double coef = (lcut - xList.get(i0-1)) / (xList.get(i0) - xList.get(i0-1));
            shortXList.add(lcut);
            shortHList.add(hList.get(i0)*coef + hList.get(i0-1)*(1-coef));
        }
        // Intermediate values
        if (iF > i0) {
            shortXList.addAll(xList.subList(i0, iF));
            shortHList.addAll(hList.subList(i0, iF));
        }
        // Last value
        if (iF < xList.size()) {
            double coef = (rcut - xList.get(iF-1)) / (xList.get(iF) - xList.get(iF-1));
            shortXList.add(rcut);
            shortHList.add(hList.get(iF)*coef + hList.get(iF-1)*(1-coef));
        }

        return integrateTrapezes(shortXList, shortHList);
    }


    /**
     * Used in overlapArea : Considers orthonormal coordinates system placing refSegment on x-axis (y = 0), usually fromNode is at (x,y) = (0,0)
     * Builds parallelogram determined by lines y = -refBufferTolerance, y = refBufferTolerance and lines determined by targetSegment buffer long sides
     * @param buffer targetSegment buffer sides
     * @return list of point Coord: anti-clockwise direction, starting from bottom left
     */
    private ArrayList<Coord> buildParallelogram(ArrayList<LocalizedVector> buffer) {
        ArrayList<Coord> parallelogram = new ArrayList<>();
        for (int i = -1; i<2; i+=2) {
            double y = refNet.getBufferTolerance()*i;
            for (int j : Arrays.asList((1+i)/2, (1-i)/2)) { // i==-1 => {0,1} ; i==1 => {1,0}
                // solving x = xM + xu*t ; y = yM + yu*t <=> t = (y - yM)/yu
                double t = (y - buffer.get(j).getFromCoord().getY()) / buffer.get(j).getVectorCoord().getY();
                parallelogram.add(new Coord(buffer.get(j).getFromCoord().getX() + buffer.get(j).getVectorCoord().getX()*t, y));
            }
        }
        return parallelogram;
    }


    /**
     * Used in overlapArea : Builds a list representing a cut in parallellogram: cuts represent short sides (withs) of refSegment buffers
     * @param x vertical cut x location
     * @param y1 vertical cut bottom point y location
     * @param y2 vertical cut top point y location
     * @return List of 2 points representing the cut
     */
    private static ArrayList<Coord> newVerticalCut(double x, double y1, double y2) {
        ArrayList<Coord> cutList = new ArrayList<>();
        cutList.add(new Coord(x, y1));
        cutList.add(new Coord(x, y2));
        return cutList;
    }


    /**
     * Used in overlapArea : Makes a symmetry of the cut around x-axis (y = 0)
     * @param cutList List containing points representing cut. Same list is to be reversed.
     */
    private static void cutVerticalSymmetry(ArrayList<Coord> cutList) {
        Coord temp = cutList.get(0);
        cutList.set(0, new Coord(cutList.get(1).getX(), -cutList.get(1).getY()));
        cutList.set(1, new Coord(temp.getX(), -temp.getY()));
    }


    /**
     * Used in overlapArea : Integrates a function f composed of a succession of segments
     * @param xList locations of points where f is evaluated
     * @param fList values of f on points of xList
     * @return integral of f
     */
    private static double integrateTrapezes(ArrayList<Double> xList, ArrayList<Double> fList) {
        if (xList.size() != fList.size()) {
            LOG.warn("xList and fList not the same size for integration, will integrate until the shortest is done");
        }
        double integral = 0;
        for (int i = 1; i < Math.min(xList.size(), fList.size()) ; i++) {
            integral += (xList.get(i) - xList.get(i-1)) * (fList.get(i) + fList.get(i-1)) / 2;
        }
        return integral;
    }


    private static int sign(int x) {
        return sign((double) x);
    }

    private static int sign(long x) {
        return sign((double) x);
    }

    private static int sign(float x) {
        return sign((double) x);
    }

    private static int sign(double x) {
        if (x<0) {
            return -1;
        } else {
            return 1;
        }
    }
}
