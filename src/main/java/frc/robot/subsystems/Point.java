package frc.robot.subsystems;

public class Point {
	public double left, right;

	public Point(double left, double right) {
			this.left = left;
			this.right = right;
	}

	public String toString() {
			return left + " : " + right;
	}
}
