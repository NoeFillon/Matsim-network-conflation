package main;

import org.matsim.api.core.v01.Coord;

import java.util.ArrayList;
import java.util.HashSet;

public class VectOp {

    public static Coord addVectors(Coord coord1, Coord coord2) {
        ArrayList<Coord> addingList = new ArrayList<>();
        addingList.add(coord1);
        addingList.add(coord2);
        return addVectors(addingList);
    }

    public static Coord addVectors(ArrayList<Coord> coordList) {
        return addVectors(new HashSet<Coord>(coordList));
    }

    public static Coord addVectors(HashSet<Coord> coordSet) {
        double x = 0; double y = 0;
        for(Coord coord : coordSet) {
            x += coord.getX();
            y += coord.getY();
        }
        return new Coord(x,y);
    }

    public static Coord extPdt(double scalar, Coord vector) {
        return new Coord(scalar* vector.getX(), scalar* vector.getY());
    }

    public static double length(Coord vector) {
        return Math.sqrt(Math.pow(vector.getX(),2) + Math.pow(vector.getY(),2));
    }

    public static double distance(Coord A, Coord B) {
        Coord AB = addVectors(A, extPdt(-1, B));
        return length(AB);
    }

    public static Coord getUnitVector(Coord vector) {
        return extPdt(1/length(vector), vector);
    }

    public static double dot(Coord vect1, Coord vect2) {
        return vect1.getX()*vect2.getX() + vect1.getY()* vect2.getY();
    }

    public static double vectorPdt(Coord vect1, Coord vect2) {
        return vect1.getX()*vect2.getY() - vect1.getY()*vect2.getX();
    }

    protected static double cosine(Coord coordl1, Coord coordl2) {
        double length1 = Math.sqrt(Math.pow(coordl1.getX(), 2) + Math.pow(coordl1.getY(), 2));
        double length2 = Math.sqrt(Math.pow(coordl2.getX(), 2) + Math.pow(coordl2.getY(), 2));
        return dot(coordl1, coordl2) / (length1 * length2);
    }

    public static Coord multiplyByMatrix(Coord line1, Coord line2, Coord vector) {
        return new Coord(dot(vector, line1), dot(vector, line2));
    }

    public static Coord piRotation(Coord vector) {
        return new Coord(-vector.getY(), vector.getX());
    }

    public static Coord expressInOrthoNBase(Coord vector, Coord u) {
        Coord i = getUnitVector(u);
        Coord step1 = multiplyByMatrix(new Coord(i.getX(), i.getY()), new Coord(-i.getY(), i.getX()), vector);
        return extPdt(1/(i.getX()*i.getX() + i.getY()*i.getY()), step1);
    }

    public static Coord expressInBase(Coord vector, Coord i, Coord j) {
        Coord step1 = multiplyByMatrix(new Coord(j.getY(), -j.getX()), new Coord(-i.getY(), i.getX()), vector);
        return extPdt(1/(i.getX()*j.getY() - i.getY()*j.getX()), step1);
    }
}
