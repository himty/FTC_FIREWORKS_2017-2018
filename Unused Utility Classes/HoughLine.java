package org.firstinspires.ftc.teamcode;

//Adaption of David Chatting's adaption.
//https://github.com/davidchatting/hough_lines
//
//Adaptation of Olly Oechsle's Hough Transform algorithm - http://vase.essex.ac.uk/software/HoughTransform/
//David Chatting - 4th March 2013
//david@davidchatting.com
//---

/**
 * Represents a linear line as detected by the hough transform.
 * This line is represented by an angle theta and a radius from the centre.
 *
 * @author Olly Oechsle, University of Essex, Date: 13-Mar-2008
 * @version 1.0
 */
public class HoughLine implements Comparable<HoughLine> {
    private double theta;
    private double r;
    private int score;

    protected int x1; //x startpoint of this line
    protected int y1; //y startpoint of this line
    protected int x2; //x endpoint of this line
    protected int y2; //y endpoint of this line


    /**
     * Initialises the hough line
     */
    public HoughLine(double theta, double r, int width, int height, int score) {
        this.theta = theta;
        this.r = r;
        this.score = score;

        // During processing h_h is doubled so that -ve r values
//        int houghHeight = (int) (Math.sqrt(2) * Math.max(height, width)) / 2;
        int houghHeight = (int) (Math.sqrt(2) * Math.max(height, width));

        // Find edge points and vote in array
        float centerX = width / 2;
        float centerY = height / 2;

        // Draw edges in output array
        double tsin = Math.sin(theta);
        double tcos = Math.cos(theta);

        if (theta < Math.PI * 0.25 || theta > Math.PI * 0.75) {
            int x1=0, y1=0;
            int x2=0, y2=height-1;

            x1=(int) ((((r - houghHeight) - ((y1 - centerY) * tsin)) / tcos) + centerX);
            x2=(int) ((((r - houghHeight) - ((y2 - centerY) * tsin)) / tcos) + centerX);

            this.x1 = x1;
            this.x2 = x2;
            this.y1 = y1;
            this.y2 = y2;
        }
        else {
            int x1=0, y1=0;
            int x2=width-1, y2=0;

            y1=(int) ((((r - houghHeight) - ((x1 - centerX) * tcos)) / tsin) + centerY);
            y2=(int) ((((r - houghHeight) - ((x2 - centerX) * tcos)) / tsin) + centerY);

            this.x1 = x1;
            this.x2 = x2;
            this.y1 = y1;
            this.y2 = y2;
        }
    }

    public int compareTo(HoughLine o) {
        return(this.score-o.score);
    }

    public String toString() {
        return "Line: x1-"+x1+" y1-"+y1+" x2-"+x2+" y2-"+y2+", theta- "+theta+" r-"+r+" Score-"+score;
    }
}

