package frc.robot.util;

import java.util.LinkedList;
import java.util.List;

public class RobotEvent {
  private List<Runnable> subscribers = new LinkedList<>();

  public void subscribe(Runnable subscriber) {
    subscribers.add(subscriber);
  }

  public void run() {
    for (Runnable subscriber : subscribers) {
      subscriber.run();
    }
  }
}
