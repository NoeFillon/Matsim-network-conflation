package main;

import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Node;
import java.lang.Math;
import java.util.Iterator;

/**
 * Preprocessed node for conflation: contains functions to know if node is terminal and keeps information
 */
public class ConflationPreprocessedNode {
    private Node node;
    private boolean isTerminal = true;


    /**
     * Constructor if one wants to force terminal of non-terminal status on node
     * @param node node
     * @param terminalByDefault status by default (true = terminal)
     */
    public ConflationPreprocessedNode(Node node, boolean terminalByDefault) {
        this.node = node;
        this.isTerminal = terminalByDefault;
    }


    /**
     * Constructor by default
     * @param node node
     * @param angleTolerance tolerance for link to be considered opposite direction: angle between links must be in [PI - angleTolerance, PI + angleTolerance]
     */
    public ConflationPreprocessedNode(Node node, double angleTolerance){
        this.node = node;
        computeIsTerminal(angleTolerance);
    }

    /**
     * Checks if this.node is terminal or not
     * @param angleTolerance tolerance for link to be considered opposite direction: angle between links must be in [PI - angleTolerance, PI + angleTolerance]
     */
    protected void computeIsTerminal(double angleTolerance) {
        this.isTerminal = true;
        if (this.node.getOutLinks().size() == 1 && this.node.getInLinks().size() == 1) {
            // One way road: one inLink and one outLink: check if the vector pair (in,out) doesn't verify in = -out
            Link inLink = this.node.getInLinks().values().iterator().next();
            Link outLink = this.node.getOutLinks().values().iterator().next();
            if (!checkOppositeDirection(inLink, outLink, angleTolerance)) {
                this.isTerminal = false;
            }
        } else if (this.node.getInLinks().size() == 2 && this.node.getOutLinks().size() == 2) {
            // Two ways road: two inLinks & two outLinks: check if there are two pairs (in,out) where the vectors verify in = -out
            Iterator<Link> inLinksIt = (Iterator<Link>) this.node.getInLinks().values().iterator();
            Iterator<Link> outLinksIt = (Iterator<Link>) this.node.getOutLinks().values().iterator();
            Link inLink1 = inLinksIt.next();
            Link inLink2 = inLinksIt.next();
            Link outLink1 = outLinksIt.next();
            Link outLink2 = outLinksIt.next();
            if ((checkOppositeDirection(inLink1, outLink1, angleTolerance) && checkOppositeDirection(inLink2, outLink2, angleTolerance)) ||
                    (checkOppositeDirection(inLink1, outLink2, angleTolerance) && checkOppositeDirection(inLink2, outLink1, angleTolerance))) {
                this.isTerminal = false;
            }
        }
    }


    /**
     * Used in computeIsTerminal : checks if links l1, l2 are opposite direction i.e. their angle is in [PI - angleTolerance, PI + angleTolerance]
     * @param l1 link 1
     * @param l2 link 2
     * @param angleTolerance tolerance for link to be considered opposite direction
     * @return : angle between links must be in [PI - angleTolerance, PI + angleTolerance]
     */
    private boolean checkOppositeDirection(Link l1, Link l2, double angleTolerance) {
        return VectOp.cosine(ConflationPreprocessedNetwork.getLinkVectorCoords(l1), ConflationPreprocessedNetwork.getLinkVectorCoords(l2)) < - Math.cos(angleTolerance);
    }


    public Node getNode() {
        return this.node;
    }


    public boolean getIsTerminal() {
        return isTerminal;
    }
}
