// Rotation and translation of object
// In the frame of the map
void recordObj(float obj_x, float obj_y)
{
	calc_x = (cos(pose_theta) * distance) + (-sin(pose_theta) * distance) + (pose_x * 1));
	calc_y = (sin(pose_theta) * distance) + (cos(pose_theta) * distance) + (pose_y * 1));
	obj_x.append(calc_x);
	obj_y.append(calc_y);
	noObj += noObj + 1;
}