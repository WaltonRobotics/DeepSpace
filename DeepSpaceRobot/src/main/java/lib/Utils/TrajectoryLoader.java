package lib.Utils;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Objects;

public class TrajectoryLoader {

  private File m_directory;
  private HashMap<String, Trajectory> m_trajMap;

  public TrajectoryLoader(String directoryFilePath) throws IOException {
    m_directory = new File(directoryFilePath);
    m_trajMap = new HashMap<>();

    try {
      for (File traj : Objects.requireNonNull(m_directory.listFiles())) {
        m_trajMap.put(traj.getName(), Pathfinder.readFromFile(traj));
      }
    }
    catch (Exception e) {
      System.out.println(e.toString() + " exception in Trajectory Loading");
    }
  }

  public Trajectory getTrajectory(String trajName) {
    return  m_trajMap.get(trajName + ".traj");
  }
}
