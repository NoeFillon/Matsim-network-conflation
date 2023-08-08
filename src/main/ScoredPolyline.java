package main;

public class ScoredPolyline {
    double score;
    private Polyline polyline;

    ScoredPolyline(Polyline polyline, double score) {
        this.polyline = polyline;
        this.score = score;
    }

    public Polyline getPolyline() {
        return polyline;
    }
}

/*
Ici on veut attribuer un score aux polylines : score(Polyline, Segment)
mais il sera peut-être nécessaire plus tard de noter des polylines avec des polylines et pas des segments
A décider une fois qu'on saura quoi faire avec les partial matches...
*/