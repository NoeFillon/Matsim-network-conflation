package main;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Coord;
import org.matsim.api.core.v01.Id;
import org.matsim.api.core.v01.network.Link;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.NetworkWriter;
import org.matsim.api.core.v01.network.Node;
import org.matsim.core.network.NetworkUtils;

import java.lang.Math;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

public class Main {

    private final static Logger LOG = LogManager.getLogger(NetworkConflator.class);

    public static void saveSimplifiedNetwork(ConflationPreprocessedNetwork preprocessedNet, String path) {
        Network simplifiedNet = preprocessedNet.createSimplifiedNetworkFromSegments();
        new NetworkWriter(simplifiedNet).write(path);
    }

    public static void main(String[] args) {
        HashSet<String> modesToKeep = new HashSet<>();
        modesToKeep.add("car");

        NetworkConflator conflator = new NetworkConflator("Entry Networks/emNetworkAm.xml", 20, 50,
                "Entry Networks/montreal_net.xml", 20, 50,
                100, modesToKeep, true, "Entry networks/montreal_emme_simplified.xml", "Entry networks/montreal_osm_simplified.xml");


        // TODO : ajouter des commentaires en tête des fonctions et des classes (cf. NetworkConflator l. 96)


        // TODO : tester toute la procédure de recherche de candidats
        //  fait : findPotentialCandidateMatches (segments compatibles),
        //          findCandidateMatches (polylignes les plus proches possibles)
        //  suivant : populateOneSegment entier


        // FIXME : vérifier le calcul des scores


        // TODO : éventuellement modifier la tolérance segment pour qu'elle dépende de la densité locale de segments ou de la longueur du segment


        // TODO : reporter les fonctions de la classe Main dans la classe NetworkConflator pour figer la construction et ne laisser la main que sur les paramètres
        //  Avertir quand la dimension du RTree est <= à la somme des tolérances buffer


        LOG.info("Looking for potential candidates");
        for (Segment refSegment : conflator.getRefNet().getSegments().values()) {
            conflator.findSegmentCandidateMatches(refSegment);
        }
        LOG.info("Done looking for potential candidates");



        /*
        HashMap<Long, HashSet<ScoredPolyline>> goodCandidateMatches = conflator.populateForNetwork();

        for (Long segmentId : goodCandidateMatches.keySet()) {
            Segment segment = preprocessedEmmeNet.getSegments().get(segmentId);
            // Exemple de Mélocheville : x € [267070, 271740[, y € [5017200, 5020000[
            boolean segmentInArea = false;
            for (Node node : segment.getAllNodes()) {
                double x = node.getCoord().getX();
                double y = node.getCoord().getY();
                if (267070 < x && x < 271740 && 5017200 < y && y < 5020000) {
                    segmentInArea = true;
                    break;
                }
            }
            if (segmentInArea) {
                System.out.print("Segment: "+segmentId+" ; Links: ");
                for (Link link : segment.getLinks()) {
                    preprocessedEmmeNet.printLink(link.getId());
                }
                System.out.println("");
                System.out.println("Candidate Polylines: ");
                for (ScoredPolyline candidate : goodCandidateMatches.get(segmentId)) {
                    System.out.print("Score: "+candidate.score+" ; links: ");
                    conflator.printLinks(candidate.getPolyline());
                }
                System.out.println("");
            }
        }
         */
    }
}