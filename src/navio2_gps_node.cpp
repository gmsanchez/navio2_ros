#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <Common/Ublox.h>
#include <Common/Util.h>
#include <cmath>

int main(int argc, char **argv) {

  std::string frame_id = "base_link";
  double sensor_frequency = 1.0;
  int solution_rate = 1000;

  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "gps");
  ros::NodeHandle nh, private_nh("~");
  ROS_INFO("Initializing %s node.", ros::this_node::getName().c_str());

  private_nh.getParam("sensor_frequency", sensor_frequency);
  private_nh.getParam("frame_id", frame_id);

  if (check_apm()) {
    return 1;
  }

  // This vector is used to store location data, decoded from ubx messages.
  // After you decode at least one message successfully, the information is stored in vector
  // in a way described in function decodeMessage(vector<double>& data) of class UBXParser(see ublox.h)

  std::vector<double> pos_data;

  // create ublox class instance
  Ublox gps;
  ROS_INFO("before publisher");
  ros::Publisher gps_pub = private_nh.advertise<sensor_msgs::NavSatFix>("fix", 10);
  //Sets the loop to publish at a rate of <sensor_frequency> Hz
  ros::Rate rate(sensor_frequency);
  solution_rate = 1000/sensor_frequency;
  ROS_INFO("Solution rate: %d", solution_rate);

  sensor_msgs::NavSatFix gps_msg;
  sensor_msgs::NavSatStatus gps_status;

  if (gps.testConnection()) {
    ROS_INFO("Ublox test OK.\n");
    if (!gps.configureSolutionRate(solution_rate))
    {
      ROS_ERROR("Setting new rate: FAILED.\n");
    }

    while(ros::ok()) {

      ros::Time current_time = ros::Time::now();
      // ROS_INFO("The time is %.4f", current_time);

      if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
      {
        gps_msg.status = gps_status;

        gps_msg.header.stamp = current_time;
        gps_msg.header.frame_id = frame_id;

        gps_msg.latitude = pos_data[2] / 10000000.0;
        gps_msg.longitude = pos_data[1] / 10000000.0;
        gps_msg.altitude = pos_data[3] / 1000.0;

        // Fill in the diagonal
        gps_msg.position_covariance[0] = pow(pos_data[5] / 1000.0, 2);
        gps_msg.position_covariance[4] = pow(pos_data[5] / 1000.0, 2);
        gps_msg.position_covariance[8] = pow(pos_data[6] / 1000.0, 2);
        gps_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

        gps_pub.publish(gps_msg);
      }

      if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1) {
        switch((int)pos_data[0]){
          case 0x00:
            // printf("no fix\n");
            gps_status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
            break;

          case 0x01:
            // printf("dead reckoning only\n");
            gps_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
            break;

          case 0x02:
            // printf("2D-fix\n");
            gps_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
            break;

          case 0x03:
            // printf("3D-fix\n");
            gps_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
            break;

          case 0x04:
            // printf("GPS + dead reckoning combined\n");
            gps_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
            break;

          case 0x05:
            // printf("Time only fix\n");
            gps_status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            gps_status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; // not sure this is true
            break;

          default:
            // printf("Reserved value. Current state unknown\n");
            break;
        }
      }
      // Pub & sleep.
      gps_pub.publish(gps_msg);
      ros::spinOnce(); // the missing call
      rate.sleep();
    }
  } else {
    ROS_ERROR("Ublox test not passed.\nAbort program!\n");
  }
  return 0;
}
