package main;

import org.matsim.api.core.v01.Coord;

import java.util.ArrayList;

public class test {
    public static void main(String[] args) {
        ArrayList<Coord> points = new ArrayList<>();
        points.add(new Coord(-1, 0));
        points.add(new Coord(-2, 1.1));
        points.add(new Coord(-1, 1));
        points.add(new Coord(0, 2));
        points.add(new Coord(-1, 2.9));
        points.add(new Coord(-0, 3));

        for (Coord point : points) {
            double t = new LocalizedVector(new Coord(0,0), new Coord(1,2)).closestPointInLocalizedVector(point);
            System.out.println(t);
        }
    }
}
