package frc.robot.subsystems;
import java.util.Scanner;
import java.util.List;
import java.io.File;
import java.util.ArrayList;


public class Auto{
	public static int counter;

	public static List<Point> readPath(){
		List<Point> points = new ArrayList<Point>();

		try {
			File file = new File("/home/lvuser/paths/path.csv");

			Scanner sc = new Scanner(file);

			while (sc.hasNext()) {
					String line = sc.next();
					String[] point = line.split(",");
					double left = Double.valueOf(point[0]);
					double right = Double.valueOf(point[1]);

					points.add(new Point(left, right));
			}

			sc.close();

		} catch (Exception e) {
			e.printStackTrace();
		}

		return points;
	}
}