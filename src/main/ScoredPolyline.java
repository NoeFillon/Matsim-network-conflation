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