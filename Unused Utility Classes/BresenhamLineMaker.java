package org.firstinspires.ftc.teamcode;

/**
 * http://www.sanfoundry.com/java-program-bresenham-line-algorithm/
 */

import java.util.ArrayList;
import java.util.List;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.Point;

public class BresenhamLineMaker
{
    /** function findLine() - to find that belong to line connecting the two points **/
    public List<Point> findLine(Bitmap grid, int x0, int y0, int x1, int y1)
    {
        List<Point> line = new ArrayList<Point>();

        int dx = Math.abs(x1 - x0);
        int dy = Math.abs(y1 - y0);

        int sx = x0 < x1 ? 1 : -1;
        int sy = y0 < y1 ? 1 : -1;

        int err = dx-dy;
        int e2;

        while (true)
        {
            line.add(new Point(x0, y0));

            if (x0 == x1 && y0 == y1)
                break;

            e2 = 2 * err;
            if (e2 > -dy)
            {
                err = err - dy;
                x0 = x0 + sx;
            }

            if (e2 < dx)
            {
                err = err + dx;
                y0 = y0 + sy;
            }
        }
        return line;
    }

    /**
     * Plots the line onto the bitmap
     * @param bm
     * @param line
     */
    public void plot(Bitmap bm, List<Point> line) {
        for (Point p : line) {
            if (p.x >= 0 && p.x < bm.getWidth()
                    && p.y >= 0 && p.y < bm.getHeight()) {
                bm.setPixel(p.x, p.y, Color.GREEN);
            }
        }
    }

    //example on how to use this class
//        Bresenham b = new Bresenham();
//
//        List<Point> line = b.findLine(grid, sr, sc, fr, fc);
//
//        b.plot(grid, line);
}
