package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.CameraStreamSource;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;

/**
 * Simple MJPEG fetcher for Limelight that implements FTC Dashboard CameraStreamSource.
 * Pulls the latest JPEG frame from Limelight's MJPEG stream URL when requested.
 */
public class LimelightDashboardStream implements CameraStreamSource {

    private final String mjpegUrl;

    public LimelightDashboardStream(String limelightHostOrIp) {
        // Typical Limelight MJPEG endpoint
        this.mjpegUrl = "http://" + limelightHostOrIp + ":5800/stream.mjpg";
    }

    @Override
    public @Nullable Bitmap getFrameBitmap() {
        // Naive approach: open MJPEG stream and parse one JPEG frame
        HttpURLConnection conn = null;
        try {
            URL url = new URL(mjpegUrl);
            conn = (HttpURLConnection) url.openConnection();
            conn.setConnectTimeout(500);
            conn.setReadTimeout(1000);
            conn.setUseCaches(false);
            conn.setDoInput(true);
            conn.connect();

            String contentType = conn.getHeaderField("Content-Type");
            if (contentType == null || !contentType.contains("multipart/x-mixed-replace")) {
                return null;
            }

            InputStream in = new BufferedInputStream(conn.getInputStream());
            // Read until we find a JPEG frame (look for JPEG SOI/EOI markers)
            // This is a simple parser; acceptable for preview usage.
            ByteArrayOutputStream buffer = new ByteArrayOutputStream(64 * 1024);

            int state = 0; // 0: seek 0xFF, 1: seek 0xD8, 2: reading until 0xFFD9
            int b;
            while ((b = in.read()) != -1) {
                if (state == 0) {
                    if (b == 0xFF) { state = 1; buffer.reset(); buffer.write(b); }
                } else if (state == 1) {
                    buffer.write(b);
                    if (b == 0xD8) { state = 2; }
                    else if (b != 0xFF) { state = 0; }
                } else {
                    buffer.write(b);
                    if (b == 0xD9) {
                        // JPEG EOI reached
                        byte[] jpeg = buffer.toByteArray();
                        return BitmapFactoryCompat.decodeByteArray(jpeg);
                    }
                }
            }
        } catch (IOException ignored) {
            // swallow; return null frame
        } finally {
            if (conn != null) conn.disconnect();
        }
        return null;
    }

    // Minimal BitmapFactory wrapper that avoids heavy options.
    private static class BitmapFactoryCompat {
        static @Nullable Bitmap decodeByteArray(byte[] data) {
            try {
                return android.graphics.BitmapFactory.decodeByteArray(data, 0, data.length);
            } catch (Throwable t) {
                return null;
            }
        }
    }
}


