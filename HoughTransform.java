package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import java.util.Collections;
import java.util.Vector;

/**
 * Hough Transform.
 * Credits to https://rosettacode.org/wiki/Hough_transform#Java
 *  - Instances of BufferedImage were changed to Bitmap
 */

public class HoughTransform
{
    ArrayData arrayData;

    private int bmHeight;
    private int bmWidth;

    public HoughTransform(Bitmap bm) {
        bmHeight = bm.getHeight();
        bmWidth = bm.getWidth();
        arrayData = getArrayDataFromBitmap(bm);
        int minContrast = 64;
        arrayData = houghTransform(arrayData, 640, 480, minContrast);
    }

    public static ArrayData houghTransform(ArrayData inputData, int thetaAxisSize, int rAxisSize, int minContrast)
    {
        int width = inputData.width;
        int height = inputData.height;
        int maxRadius = (int)Math.ceil(Math.hypot(width, height));
        int halfRAxisSize = rAxisSize >>> 1;
        ArrayData outputData = new ArrayData(thetaAxisSize, rAxisSize);
        // x output ranges from 0 to pi
        // y output ranges from -maxRadius to maxRadius
        double[] sinTable = new double[thetaAxisSize];
        double[] cosTable = new double[thetaAxisSize];
        for (int theta = thetaAxisSize - 1; theta >= 0; theta--)
        {
            double thetaRadians = theta * Math.PI / thetaAxisSize;
            sinTable[theta] = Math.sin(thetaRadians);
            cosTable[theta] = Math.cos(thetaRadians);
        }

        for (int y = height - 1; y >= 0; y--)
        {
            for (int x = width - 1; x >= 0; x--)
            {
                if (inputData.contrast(x, y, minContrast))
                {
                    for (int theta = thetaAxisSize - 1; theta >= 0; theta--)
                    {
                        double r = cosTable[theta] * x + sinTable[theta] * y;
                        int rScaled = (int)Math.round(r * halfRAxisSize / maxRadius) + halfRAxisSize;
                        outputData.accumulate(theta, rScaled, 1);
                    }
                }
            }
        }
        return outputData;
    }

    public static class ArrayData
    {
        public final int[] dataArray;
        public final int width;
        public final int height;

        public ArrayData(int width, int height)
        {
            this(new int[width * height], width, height);
        }

        public ArrayData(int[] dataArray, int width, int height)
        {
            this.dataArray = dataArray;
            this.width = width;
            this.height = height;
        }

        public int get(int x, int y)
        {  return dataArray[y * width + x];  }

        public void set(int x, int y, int value)
        {  dataArray[y * width + x] = value;  }

        public void accumulate(int x, int y, int delta)
        {  set(x, y, get(x, y) + delta);  }

        public boolean contrast(int x, int y, int minContrast)
        {
            int centerValue = get(x, y);
            for (int i = 8; i >= 0; i--)
            {
                if (i == 4)
                    continue;
                int newx = x + (i % 3) - 1;
                int newy = y + (i / 3) - 1;
                if ((newx < 0) || (newx >= width) || (newy < 0) || (newy >= height))
                    continue;
                if (Math.abs(get(newx, newy) - centerValue) >= minContrast)
                    return true;
            }
            return false;
        }

        public int getMax()
        {
            int max = dataArray[0];
            for (int i = width * height - 1; i > 0; i--)
                if (dataArray[i] > max)
                    max = dataArray[i];
            return max;
        }
    }

    public static ArrayData getArrayDataFromBitmap(Bitmap bm) {
        int width = bm.getWidth();
        int height = bm.getHeight();
        ArrayData arrayData = new ArrayData(width, height);
        // Flip y axis when reading image
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                int rgbValue = bm.getPixel(x,y);
                rgbValue = (int)(((rgbValue & 0xFF0000) >>> 16) * 0.30 + ((rgbValue & 0xFF00) >>> 8) * 0.59 + (rgbValue & 0xFF) * 0.11);
                arrayData.set(x, height - 1 - y, rgbValue);
            }
        }

        return arrayData;
    }

    public static Bitmap getBitmapFromArrayData(ArrayData arrayData) {
        int max = arrayData.getMax();
        Bitmap outputbm = Bitmap.createBitmap(arrayData.width, arrayData.height, Bitmap.Config.RGB_565);
        for (int y = 0; y < arrayData.height; y++)
        {
            for (int x = 0; x < arrayData.width; x++)
            {
                int n = Math.min((int)Math.round(arrayData.get(x, y) * 255.0 / max), 255);
                outputbm.setPixel(x, arrayData.height - 1 - y, (n << 16) | (n << 8) | 0x90 | -0x01000000);
            }
        }

        return outputbm;
    }

    public Bitmap getOutputBitmap() {
        return getBitmapFromArrayData(arrayData);
    }

    /*
     * Returns lines calculated after performing the Hough Transform
     * @param n             the number of lines to find
     * @param threshold     minimum score of each returned line
     *
     * Adaption of David Chatting's algorithm
     * https://github.com/davidchatting/hough_lines/blob/master/HoughTransform.java
     */
    public Vector<HoughLine> getLines(int n, int threshold) {
        int neighbourhoodSize = 20;
        int maxTheta = 640;

        // Using maxTheta, work out the step
        final double thetaStep = Math.PI / maxTheta;

        // Initialise the vector of lines that we'll return
        Vector<HoughLine> lines = new Vector<HoughLine>(20);

        // Only proceed if the hough array is not empty
        if (arrayData.width == 0) {
            return lines;
        }

        // Search for local peaks above threshold to draw
        for (int t = 0; t < arrayData.width; t++) {
            loop:
            for (int r = neighbourhoodSize; r < arrayData.height - neighbourhoodSize; r++) {

                // Only consider points above threshold
                if (arrayData.get(t, r) > threshold) {

                    int peak = arrayData.get(t, r);

                    // Check that this peak is indeed the local maxima
                    for (int dx = -neighbourhoodSize; dx <= neighbourhoodSize; dx++) {
                        for (int dy = -neighbourhoodSize; dy <= neighbourhoodSize; dy++) {
                            int dt = t + dx;
                            int dr = r + dy;
                            if (dt < 0) dt = dt + maxTheta;
                            else if (dt >= maxTheta) dt = dt - maxTheta;
                            if (arrayData.get(dt, dr) > peak) {
                                // found a bigger point nearby, skip
                                continue loop;
                            }
                        }
                    }

                    // calculate the true value of theta
                    double theta_scaled = t * thetaStep;
//                    double r_scaled = r * bmHeight/480;

                    // add the line to the vector
                    lines.add(new HoughLine(theta_scaled, r, arrayData.width, arrayData.height, arrayData.get(t, r)));
                }
            }
        }
        Collections.sort(lines, Collections.reverseOrder());
        lines.setSize(n);

        return lines;
    }
}

//Example usage of this class
//        ArrayData inputData = getArrayDataFromBitmap(bm);
//        int minContrast = (args.length >= 4) ? 64 : Integer.parseInt(args[4]);
//        ArrayData outputData = houghTransform(inputData, Integer.parseInt(args[2]), Integer.parseInt(args[3]), minContrast);
//        Bitmap bm = getBitmapFromArrayData(outputData);