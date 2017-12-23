package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.Frame;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

/**
 * Created by jenny on 12/22/2017.
 */

public class VuforiaLocalizerImplSubclass extends VuforiaLocalizerImpl {

    public Image rgb;

    class ClosableFrame extends Frame {
        public ClosableFrame(Frame other) {
            super(other);
        }

        public void close() {
            super.delete();
        }
    }

    public class VuforiaCallbackSubclass extends VuforiaLocalizerImpl.VuforiaCallback {
        @Override
        public synchronized void Vuforia_onUpdate(State state) {
            super.Vuforia_onUpdate(state);

            VuforiaLocalizer.CloseableFrame frame = new VuforiaLocalizer.CloseableFrame(state.getFrame());
//            RobotLog.vv("VuforiaLocalizerImplSubclass", "received Vuforia frame#=%d", state.getFrame());

            long num = frame.getNumImages();

            for (int i = 0; i < num; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                }
            }

            frame.close();
        }
    }

    public VuforiaLocalizerImplSubclass(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
        stopAR();
        clearGlSurface();

        this.vuforiaCallback = new VuforiaCallbackSubclass();
        startAR();

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
    }

    public void clearGlSurface() {
        if (this.glSurfaceParent != null) {
            appUtil.synchronousRunOnUiThread(new Runnable() {
                @Override
                public void run() {
                    glSurfaceParent.removeAllViews();
                    glSurfaceParent.getOverlay().clear();
                    glSurface = null;
                }
            });
        }
    }
}
