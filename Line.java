package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

/**
 * Created by jenny on 2/7/2018.
 */

public class Line {
    Point myPoint;
    double mySlope;

    public Line(Point pt, double slope) {
        myPoint = pt;
        mySlope = slope;
    }

    /**
     * @return a point on this line
     */
    public Point getPoint() {
        return myPoint;
    }

    /*
     * @return the slope of this line
     */
    public double getSlope() {
        return mySlope;
    }
}
