package main;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.network.Network;
import org.matsim.api.core.v01.network.NetworkWriter;

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

        NetworkConflator conflator = new NetworkConflator("EntryNetworks/emNetworkAm.xml", 20, 50, true,
                "EntryNetworks/montreal_net.xml", 20,
                50, false, 100, modesToKeep, true, "EntryNetworks/montreal_emme_simplified.xml", "EntryNetworks/montreal_osm_simplified.xml");


        // TODO : tester toute la procédure de recherche de candidats
        //  fait : findPotentialCandidateMatches (segments compatibles),
        //          findCandidateMatches (polylignes les plus proches possibles)
        //  suivant : populateOneSegment entier


        // FIXME : vérifier le calcul des scores


        // TODO : éventuellement modifier la tolérance segment pour qu'elle dépende de la densité locale de segments ou de la longueur du segment


        // TODO : reporter les fonctions de la classe Main dans la classe NetworkConflator pour figer la construction et ne laisser la main que sur les paramètres
        //  Avertir quand la dimension du RTree est <= à la somme des tolérances buffer


        HashMap<Long, HashSet<ScoredPolyline>> networkCandidates = conflator.populateForNetwork();

        for(Long id : networkCandidates.keySet()) {
            System.out.print("Segment " + id + " : ");
            for(ScoredPolyline goodCandidate : networkCandidates.get(id)) {
                System.out.print(goodCandidate.score + " ; ");
            }
            System.out.println("");
        }


        // Exemple de Mélocheville : x € [267070, 271740[, y € [5017200, 5020000[
    }
}