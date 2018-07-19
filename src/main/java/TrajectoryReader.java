import java.io.*;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryReader {

    private File file;

    public TrajectoryReader(String filename) {
        String fullPath = TrajectoryReader.class.getResource(filename).getPath();
        file = new File(fullPath);
        System.out.println(file.getPath());
    }

    public List<Trajectory> readTrajectories() {
        List<Trajectory> trajectories = new ArrayList<>();

        try {
            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line = reader.readLine();
            Trajectory t = null;
            int currentId = -1;
            while((line = reader.readLine()) != null) {
                String[] split = line.split(";");
                int taxiId = Integer.valueOf(split[0]);
                int id = Integer.valueOf(split[1]);
                double lat = Double.valueOf(split[2]);
                double lng = Double.valueOf(split[3]);
                long timestamp = Long.valueOf(split[4]);
                if (currentId == -1)
                    t = new Trajectory(id, taxiId);
                else if (id != currentId) {
                    trajectories.add(t);
                    t = new Trajectory(id, taxiId);
                }
                t.addPoint(lat, lng, timestamp);
                currentId = id;
            }
            trajectories.add(t);
        } catch (IOException e) {
            e.printStackTrace();
        }

        return trajectories;
    }

}
