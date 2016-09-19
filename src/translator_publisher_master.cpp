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



class TranslatorPublisher
{
public:
  TranslatorPublisher(int jrp, int frp) : Joint_recv_port(jrp), Force_recv_port(frp)
  {


   pub_joints = n_.advertise<sensor_msgs::JointState>("/slave/joint_states", 1000);
   pub_force = n_.advertise<geometry_msgs::Wrench>("/slave/wrench_r", 1000);


   struct sockaddr_in local;

   // Initialize incoming network connection for joint
   sr_joint = socket(AF_INET, SOCK_DGRAM, 0);
   if (sr_joint < 0) {
       printf("Error opening sr_joint socket!\n");
       exit(1);
   }

   local.sin_family = AF_INET;
   local.sin_port = htons(Joint_recv_port);
   local.sin_addr.s_addr = INADDR_ANY;

   if (bind(sr_joint, (struct sockaddr *)&local, sizeof(local)) < 0) {
       printf("Error binding sr_joint!\n");
       exit(1);
   }

   // Initialize incoming network connection for force
   sr_force = socket(AF_INET, SOCK_DGRAM, 0);
   if (sr_joint < 0) {
       printf("Error opening sr_force socket!\n");
       exit(1);
   }

   local.sin_family = AF_INET;
   local.sin_port = htons(Force_recv_port);
   local.sin_addr.s_addr = INADDR_ANY;

   if (bind(sr_force, (struct sockaddr *)&local, sizeof(local)) < 0) {
       printf("Error binding sr_force!\n");
       exit(1);
   }
  }

  void netRecvJoint()
  {
      int ret;
      char buf[1024];
      struct sockaddr_in  from;
      socklen_t           from_len = sizeof(from);

      // receive data
      ROS_INFO("receiving slave joints from outer network and publishing them in ROS:");


      ret = recvfrom(sr_joint, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
      if (ret <= 0)
          printf("Receive Error: ret = %d\n", ret);

      //deserilize data
      sensor_msgs::JointState msg_out;
     // uint32_t serial_size = ros::serialization::serializationLength(msg_out);
      boost::shared_array<uint8_t> buffer(new uint8_t[ret]);

      memcpy(buffer.get(), (void *)buf, ret);

      ros::serialization::IStream stream(buffer.get(), ret);
      ros::serialization::deserialize(stream, msg_out);

      // publish received and deserialized data to ROS
      pub_joints.publish(msg_out);
  }

  void netRecvForce()
  {
      int ret;
      char buf[1024];
      struct sockaddr_in  from;
      socklen_t           from_len = sizeof(from);

      // receive
      ROS_INFO("receiving slave force from outer network and publishing them in ROS:");


      ret = recvfrom(sr_force, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
      if (ret <= 0)
          printf("Receive Error: ret = %d\n", ret);

      //deserilize data
      geometry_msgs::Wrench msg_out;
     // uint32_t serial_size = ros::serialization::serializationLength(msg_out);
      boost::shared_array<uint8_t> buffer(new uint8_t[ret]);

      memcpy(buffer.get(), (void *)buf, ret);

      ros::serialization::IStream stream(buffer.get(), ret);
      ros::serialization::deserialize(stream, msg_out);

      // publish received and deserialized data to ROS
      pub_force.publish(msg_out);
  }

  int sr_joint, sr_force;

private:
  ros::NodeHandle n_;
  ros::Publisher pub_force;
  ros::Publisher pub_joints;

  //define sockets here
  int Joint_recv_port;
  int Force_recv_port;

};

void Usage (int argc, char *argv[], int *jrp, int *frp)
{
    int i, i1, i2, i3, i4;

    // Global Variable Initialization
    *jrp = 20020;
    *frp = 20021;

    while (--argc > 0) {
        argv++;

        if (!strncmp(*argv, "-jr", 3)) {
            sscanf(argv[1], "%d", jrp);
            argc--; argv++;
        } else if (!strncmp(*argv, "-fr", 3)) {
            sscanf(argv[1], "%d", frp);
            argc--; argv++;
        } else {
            printf("\nUsage: \n"
                "\t[-jr <port number>] : port on which to receive joint data\n"
                "\t[-fr <port number>] : port on which to receive force data\n"
                "\n");
            exit(1);
        }
    }

    printf("\nStarting execution with the following settings:\n"
        "\tJoint Recv Port = %d\n"
        "\tForce Recv Port = %d\n"
        "\n",
        *jrp, *frp);
}

int main(int argc, char **argv)
{
  int jrp, frp;
  fd_set mask, temp_mask, dummy_mask;
  int quit = 0;
  int num;

  ros::init(argc, argv, "translator_publisher");
  Usage(argc, argv, &jrp, &frp);

  TranslatorPublisher translatorObject(jrp, frp);

  // Setup Masks for Feedback Loop
  FD_ZERO(&mask);
  FD_ZERO(&dummy_mask);
  FD_SET(translatorObject.sr_joint, &mask);      /* joint socket */
  FD_SET(translatorObject.sr_force, &mask);      /* force socket */
  FD_SET((long)0, &mask);         /* stdin for quit signal */

  printf("Press enter quit...\n");

  while (!quit) {
      temp_mask = mask;
      num = select(FD_SETSIZE, &temp_mask, &dummy_mask, &dummy_mask, NULL);

      if (num > 0) {
          if (FD_ISSET(0, &temp_mask))
              quit = 1;
          else if (FD_ISSET(translatorObject.sr_joint, &temp_mask))
              translatorObject.netRecvJoint();
          else if (FD_ISSET(translatorObject.sr_force, &temp_mask))
              translatorObject.netRecvForce();
      }
  }

  return 0;
}
