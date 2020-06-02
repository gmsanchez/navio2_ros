#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "Common/MPU9250.h"
#include "Navio2/LSM9DS1.h"
#include "Common/Util.h"
#include <unistd.h>
#include <string>
#include <memory>

std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name)
{
    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
}

void rotate_rep103(float &_ax, float &_ay, float &_az,
                   float &_gx, float &_gy, float &_gz,
                   float &_mx, float &_my, float &_mz)
{
  float replacement_acc, replacement_gyro;

  replacement_acc = _ax;
  _ax = -_ay;
  _ay = -replacement_acc;

  replacement_gyro = _gx;
  _gx = -_gy;
  _gy = -replacement_gyro;

  _mx = -_mx;
  _my = -_my;
  _mz = -_mz;
}

int main(int argc, char **argv) {

  std::string sensor_name, frame_id;
  double sensor_frequency = 10.0;

  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "imu");
  ros::NodeHandle nh, private_nh("~");
  ROS_INFO("Initializing %s node.", ros::this_node::getName().c_str());

  private_nh.getParam("sensor_name", sensor_name);
  private_nh.getParam("sensor_frequency", sensor_frequency);
  private_nh.param<std::string>("frame_id", frame_id, "imu_link");
  /*
  http://wiki.ros.org/roscpp_tutorials/Tutorials/Parameters
  n.param("my_num", i, 42);
  n.param<std::string>("my_param", s, "default_value");
  if (n.getParam("my_param", s))
    {
      ROS_INFO("Got param: %s", s.c_str());
    }
    else
    {
      ROS_ERROR("Failed to get param 'my_param'");
    }
  */

  ROS_INFO("Sensor name: %s - Sensor frequency %f", sensor_name.c_str(), sensor_frequency);

  if (check_apm()) {
    return 1;
  }
  auto sensor = get_inertial_sensor(sensor_name);

  if (!sensor) {
    printf("Wrong sensor name. Select: mpu or lsm\n");
    return EXIT_FAILURE;
  }

  if (!sensor->probe()) {
    printf("Sensor not enabled\n");
    return EXIT_FAILURE;
  }
  sensor->initialize();

  ros::Publisher imu_pub = private_nh.advertise<sensor_msgs::Imu>("data_raw", 10);
  ros::Publisher mag_pub = private_nh.advertise<sensor_msgs::MagneticField>("mag", 10);

  //Sets the loop to publish at a rate of <sensor_frequency> Hz
  ros::Rate rate(sensor_frequency);
  while(ros::ok()) {

    // ROS_INFO("The time is %f", ros::Time::now());
    ros::Time current_time = ros::Time::now();
    sensor->update();
    sensor->read_accelerometer(&ax, &ay, &az);
    sensor->read_gyroscope(&gx, &gy, &gz);
    sensor->read_magnetometer(&mx, &my, &mz);

    rotate_rep103(ax, ay, az, gx, gy, gz, mx, my, mz);

    // printf("Acc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
    // printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
    // printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);

    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = frame_id;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    sensor_msgs::MagneticField mag_msg;
    mag_msg.header.stamp = current_time;
    mag_msg.header.frame_id = frame_id;

    mag_msg.magnetic_field.x = mx;
    mag_msg.magnetic_field.y = my;
    mag_msg.magnetic_field.z = mz;

    // Pub & sleep.
    imu_pub.publish(imu_msg);
    mag_pub.publish(mag_msg);
    ros::spinOnce(); // the missing call
    rate.sleep();
  }
  return 0;
}
