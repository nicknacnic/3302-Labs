#include <sparki.h>

#define ROBOT_SPEED 0.0275
#define CYCLE_TIME .100
#define AXLE_DIAMETER 0.085
#define CONTROLLER_FOLLOW_LINE 1
#define CONTROLLER_DISTANCE_MEASURE 2
#define FWD 1
#define NONE 0
#define BCK -1

int current_state = CONTROLLER_FOLLOW_LINE;
const int threshold = 700;
int line_left = 1000;
int line_center = 1000;
int line_right = 1000;

float pose_x = 0., pose_y = 0., pose_theta = 0.;
float distance = 0.;
int left_wheel_rotating, right_wheel_rotating;

void setup() {
  // put your setup code here, to run once:
  pose_x = 0.;
  pose_y = 0.;
  pose_theta = 0.;
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
}

void readSensors() {
  line_left = sparki.lineLeft();
  line_right = sparki.lineRight();
  line_center = sparki.lineCenter();
  distance = sparki.ping();
}

void measure_30cm_speed() {
  // 10773ms per 30cm = 0.0278 m/s
  unsigned long begin_time, end_time;
  sparki.clearLCD();
  begin_time = millis();
  sparki.moveForward(30);
  end_time = millis();
  sparki.print("Millis Elapsed: ");
  sparki.println(end_time - begin_time);
  sparki.updateLCD();
  sparki.moveStop();
  delay(1000);
}

void moveRight() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = BCK;
  sparki.moveRight();
}

void moveLeft() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = FWD;
  sparki.moveLeft();
}

void moveForward() {
  left_wheel_rotating = FWD;
  right_wheel_rotating = FWD;
  sparki.moveForward();
}

void moveBackward() {
  left_wheel_rotating = BCK;
  right_wheel_rotating = BCK;
  sparki.moveBackward();
}

void moveStop() {
  left_wheel_rotating = NONE;
  right_wheel_rotating = NONE;
  sparki.moveStop();
}

void updateOdometry() {
  pose_x += cos(pose_theta) * CYCLE_TIME * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_y += sin(pose_theta) * CYCLE_TIME * ROBOT_SPEED * (left_wheel_rotating + right_wheel_rotating)/2.;
  pose_theta += (right_wheel_rotating - left_wheel_rotating) * 2. * CYCLE_TIME * ROBOT_SPEED / AXLE_DIAMETER;
}

void displayOdometry() {
  sparki.println("Pose: ");
  sparki.print("X: ");
  sparki.println(pose_x);
  sparki.print("Y: ");
  sparki.println(pose_y);
  sparki.print("T: ");
  sparki.println(pose_theta * 180. / 3.14159);
  sparki.updateLCD();
}

void loop() {
  unsigned long begin_time = millis();
  unsigned long begin_movement_time = 0;
  unsigned long end_time = 0;
  unsigned long delay_time = 0;

  sparki.clearLCD();
  updateOdometry();
  displayOdometry();


  readSensors();
  
  switch (current_state) {
    case CONTROLLER_FOLLOW_LINE:
      if (line_center < threshold) {
        moveForward();
      } else if (line_left < threshold) {
        moveLeft();
      } else if (line_right < threshold) {
        moveRight();
      } else {
        moveStop();
      }

      // Check for start line, use as loop closure
      if (line_left < threshold && line_right < threshold && line_center < threshold) {
        pose_x = 0.;
        pose_y = 0.;
        pose_theta = 0.;
      } 
      break;
    case CONTROLLER_DISTANCE_MEASURE:
      measure_30cm_speed();
      break;
  }


  end_time = millis();
  delay_time = end_time - begin_time;
  if (delay_time < 1000*CYCLE_TIME)
    delay(1000*CYCLE_TIME - delay_time); // each loop takes CYCLE_TIME ms
  else
    delay(1);
}
