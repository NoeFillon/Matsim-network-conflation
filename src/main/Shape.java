package main;

import org.matsim.api.core.v01.Coord;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Stack;

public class Shape {
    private HashMap<Integer, Coord> verticesCoordinates;
    private HashMap<Integer, HashSet<Integer>> linkedVertices;

    public Shape() {
        verticesCoordinates = new HashMap<Integer, Coord>();
        linkedVertices = new HashMap<Integer, HashSet<Integer>>();
    }

    void addVertice(int i, Coord coord) {
        this.verticesCoordinates.put(i, coord);
        this.linkedVertices.put(i, new HashSet<Integer>());
    }

    void addSide(int i1, int i2) {
        this.linkedVertices.get(i1).add(i2);
        this.linkedVertices.get(i2).add(i1);
    }

    void replaceVerticeCoord(int i, Coord coord) {
        this.verticesCoordinates.replace(i, coord);
    }

    boolean containsVertice(int i) {
        return verticesCoordinates.containsKey(i);
    }

    HashMap<Integer, Coord> getCoordinatesMap() {
        return verticesCoordinates;
    }

    Coord getVerticeCoord(int i) {
        return verticesCoordinates.get(i);
    }

    HashSet<Integer> getLinkedVertices(int i) {
        return this.linkedVertices.get(i);
    }

    int findMinimalX(HashSet<Integer> indexSet) {
        int minIndex = -1;
        Double minX = null;
        for (int i : indexSet) {
            if (minX == null || this.verticesCoordinates.get(i).getX() < minX) {
                minX = this.verticesCoordinates.get(i).getX();
                minIndex = i;
            }
        }
        return minIndex;
    }

    ArrayList<Double> getWidthProfile(ArrayList<Double> xList) {
        ArrayList<Double> profile = new ArrayList<>();

        int firstIndex = this.findMinimalX(new HashSet<>(this.getCoordinatesMap().keySet()));
        ArrayList<Stack<Integer>> paths = new ArrayList<>();
        paths.add(new Stack<Integer>()); // Top path
        paths.add(new Stack<Integer>()); // Bottom path
        paths.get(0).push(firstIndex);
        paths.get(1).push(firstIndex);
        profile.add((double) 0);
        xList.add(this.getVerticeCoord(firstIndex).getX());

        // Add the vertices on top and bottom
        ArrayList<Integer> firstLinkExits = new ArrayList<>(this.getLinkedVertices(firstIndex));
        int topBottomOrder = 0;
        if (firstLinkExits.get(1) < firstLinkExits.get(0)) {
            topBottomOrder = 1;
        }
        paths.get(0).push(firstLinkExits.get(topBottomOrder));
        paths.get(1).push(firstLinkExits.get(1-topBottomOrder));

        while (paths.get(0).peek() != paths.get(1).peek()) {
            int i = 0;
            if (getVerticeCoord(paths.get(1).peek()).getX() < getVerticeCoord(paths.get(0).peek()).getX()) {
                i = 1;
            }

            // Adding a new profile point
            Coord firstPathCoord = getVerticeCoord(paths.get(i).peek());
            Coord secondPathCoordR = getVerticeCoord(paths.get(1-i).peek());
            Coord secondPathCoordL = getVerticeCoord(paths.get(1-i).get(paths.get(1-i).size()-2));
            double coef = (firstPathCoord.getX() - secondPathCoordL.getX()) / (secondPathCoordR.getX() - secondPathCoordL.getX());
            profile.add(Math.abs(firstPathCoord.getY() - (coef*secondPathCoordR.getY() + (1-coef)*secondPathCoordL.getY())));
            xList.add(firstPathCoord.getX());

            // Finding and stacking the next point on the path
            ArrayList<Integer> linkExits = new ArrayList<>(this.getLinkedVertices(paths.get(i).peek()));
            int maxXIndex = 0;
            if (getVerticeCoord(linkExits.get(1)).getX() > getVerticeCoord(linkExits.get(0)).getX()) {
                maxXIndex = 1;
            }
            paths.get(i).push(linkExits.get(maxXIndex));
        }
        profile.add(0.0);
        xList.add(getVerticeCoord(paths.get(0).peek()).getX());
        return profile;
    }
}
