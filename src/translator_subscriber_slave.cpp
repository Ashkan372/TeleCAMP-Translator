#include "ros/ros.h"
#include "std_msgs/String.h"
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Wrench.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netdb.h>
#include <errno.h>

#include <boost/shared_array.hpp>

/* Macros */
#define IPF "%d.%d.%d.%d"
#define IP1(addr)   ( (int) ( ( (addr) >> 24 ) & 0xFF ) )
#define IP2(addr)   ( (int) ( ( (addr) >> 16 ) & 0xFF ) )
#define IP3(addr)   ( (int) ( ( (addr) >>  8 ) & 0xFF ) )
#define IP4(addr)   ( (int) ( ( (addr) >>  0 ) & 0xFF ) )
#define IP(addr) IP1(addr), IP2(addr), IP3(addr), IP4(addr)



class TranslatorSubscriber
{
public:
  TranslatorSubscriber(int jsp, int fsp, long dst_ip) : Joint_send_port(jsp), Force_send_port(fsp), Dest_ip(dst_ip)
  {


   sub_joints = n_.subscribe("/slave/joint_states", 1000, &TranslatorSubscriber::jointCallback, this);
   sub_force = n_.subscribe("/slave/wrench_r", 1000, &TranslatorSubscriber::forceCallback, this);
   // Initalize outgoing network connection
   ss = socket(AF_INET, SOCK_DGRAM, 0);
   if (ss < 0) {
       printf("Error opening ss socket!\n");
       exit(1);
   }

   dst_joint.sin_family = AF_INET;
   dst_joint.sin_port = htons(Joint_send_port);
   dst_joint.sin_addr.s_addr = htonl(Dest_ip);

   dst_force.sin_family = AF_INET;
   dst_force.sin_port = htons(Force_send_port);
   dst_force.sin_addr.s_addr = htonl(Dest_ip);
  }

  void jointCallback(const sensor_msgs::JointState& msg)
  {
      int ret;
      char buf[1024];
      struct sockaddr_in  from;
      socklen_t           from_len = sizeof(from);

      ROS_INFO("sending slave joints to outer network:");

      //serilize ros message_t
      uint32_t serial_size = ros::serialization::serializationLength(msg);
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
      ros::serialization::OStream stream(buffer.get(), serial_size);
      ros::serialization::serialize(stream, msg);


      ret = sendto(ss, buffer.get(), serial_size, 0, (struct sockaddr *)&dst_joint, sizeof(dst_joint));
      if (ret != serial_size)
          printf("Send Error in masterJoint: ret = %d, len = %u\n", ret, serial_size);
  }

  void forceCallback(const geometry_msgs::Wrench& msg)
  {
      int ret;

      ROS_INFO("sending slave force to outer network:");

      //serilize ros message_t
      uint32_t serial_size = ros::serialization::serializationLength(msg);
      boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
      ros::serialization::OStream stream(buffer.get(), serial_size);
      ros::serialization::serialize(stream, msg);


      ret = sendto(ss, buffer.get(), serial_size, 0, (struct sockaddr *)&dst_force, sizeof(dst_force));
      if (ret != serial_size)
          printf("Send Error in masterJoint: ret = %d, len = %u\n", ret, serial_size);
  }

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_force;
  ros::Subscriber sub_joints;

  
  int                 ss;
  struct sockaddr_in  dst_joint, dst_force;
  long Dest_ip;
  int Joint_send_port;
  int Force_send_port;

};

void Usage (int argc, char *argv[], int *jsp, int *fsp, long *dst_ip)
{
    int i, i1, i2, i3, i4;

    // Global Variable Initialization
    *dst_ip = (127 << 24) | (0 << 16) | (0 << 8) | 1;
    *jsp = 30020;
    *fsp = 30021;

    while (--argc > 0) {
        argv++;

        if (!strncmp(*argv, "-a", 2)) {
            sscanf(argv[1], "%d.%d.%d.%d", &i1, &i2, &i3, &i4);
            *dst_ip = ( (i1<<24) | (i2<<16) | (i3<<8) | (i4) );
            argc--; argv++;
        } else if (!strncmp(*argv, "-js", 3)) {
            sscanf(argv[1], "%d", jsp);
            argc--; argv++;
        } else if (!strncmp(*argv, "-fs", 3)) {
            sscanf(argv[1], "%d", fsp);
            argc--; argv++;
        } else {
            printf("\nUsage: \n"
                "\t[-a <ip address> ]  : remote machine address\n"
                "\t[-js <port number>] : port on which to send joint data\n"
                "\t[-fs <port number>] : port on which to send force data\n"
                "\n");
            exit(1);
        }
    }

    printf("\nStarting execution with the following settings:\n"
        "\tRemote_ip = " IPF "\n"
        "\tJoint Send Port = %d\n"
        "\tForce Send Port = %d\n"
        "\n",
        IP(*dst_ip), *jsp, *fsp);
}

int main(int argc, char **argv)
{
  int jsp, fsp;
  long dst_ip;

  ros::init(argc, argv, "translator_subscriber");
  Usage(argc, argv, &jsp, &fsp, &dst_ip);

  TranslatorSubscriber translatorObject(jsp, fsp, dst_ip);

  ros::spin();

  return 0;
}
