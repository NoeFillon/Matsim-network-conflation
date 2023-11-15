package main;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.matsim.api.core.v01.Coord;
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
                50, false, 100, modesToKeep, true,
                "EntryNetworks/montreal_emme_simplified.xml", "EntryNetworks/montreal_osm_simplified.xml");

        HashMap<Long, HashSet<ScoredPolyline>> networkCandidates = conflator.populateForNetwork();

        // TODO : the way segment buffer tolerance works could be modified to depend on local density of segments or segment's length

        // Exemple de Mélocheville : x € [267070, 271740[, y € [5017200, 5020000[

    }
}