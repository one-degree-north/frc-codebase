package frc.lib.basesubsystem;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Based on code from this repository:
 * https://github.com/vayun-mathur/oak-raspberry-network
 * Update this code from that repo when nessecary
 */
public class OakSubsystem extends SubsystemBase {

    public static class Detection {
        String label;
        int x1, y1, x2, y2;
        double x, y, z;
    }

    private BufferedReader m_read;

    public OakSubsystem(String hostname, int port) {
        Socket so;
        try {
            so = new Socket(hostname, port);
            m_read = new BufferedReader(new InputStreamReader(so.getInputStream()));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void evaluateDetections(Detection[] detections) {
        
    }

    @Override
    public void periodic() {
        try {
            if (m_read.ready()) {
                String str = m_read.readLine();
                String[] objs = str.split(";");
                Detection[] dec = new Detection[objs.length];
                int i = 0;
                for(String obj: objs) {
                    String[] arr = obj.split(",");
                    Detection d = new Detection();
                    d.label = arr[0];
                    d.x1 = Integer.parseInt(arr[1]);
                    d.y1 = Integer.parseInt(arr[2]);
                    d.x2 = Integer.parseInt(arr[3]);
                    d.y2 = Integer.parseInt(arr[4]);
                    d.x = Double.parseDouble(arr[5]);
                    d.y = Double.parseDouble(arr[6]);
                    d.z = Double.parseDouble(arr[7]);
                    dec[i++] = d;
                }
                evaluateDetections(dec);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
}
