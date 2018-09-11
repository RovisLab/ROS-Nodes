#include <ros/ros.h>
#include <coords_msgs/Coords.h>
#include <darknet_ros_msgs/BoundingBoxes.h>


using namespace std;


ros::Publisher pub;
string st1;
coords_msgs::Coords coordonate;
void CoordsCallback(const darknet_ros_msgs::BoundingBoxes& msg)
{
    st1="person";
    darknet_ros_msgs::BoundingBox inauntru;
     for (int i=0; i <int(msg.bounding_boxes.size()); i++) {
        if (st1.compare(msg.bounding_boxes[i].Class) == 0)
        {
            coordonate.goal_x = msg.bounding_boxes[i].X;
            coordonate.goal_y = msg.bounding_boxes[i].Z;//or z
            ROS_INFO("Se transmit coordonatele %f, %f",coordonate.goal_x,coordonate.goal_y);
            pub.publish(coordonate);
        }
     }

    //verifici datele din mesaj.

    //datele de verificat sunt: label-ul mesajului(denumirea obiectului detectat). string clasa
    //                          probabilitatea
    //de trimis catre topicul nostru, x si z din mesajul acesta.


    //also.ar fi fain sa nu verifice de fiecare data daca se detecteaza obiectul, ci doar cand dorim noi.
    //chestia asta o putem face din main. Sau pur si simplu daca s-a trimis un mesaj, sa nu mai urmeze si altele.cauta info despre ros::spin, pt asta.


}
// create a subscriber function
//inside it check for the info and the create a mesage to send to the topic.
int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_sender");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes",10,CoordsCallback);
    pub = nh.advertise<coords_msgs::Coords>("coord_topic", 10,true);

    ros::spin();

    return 0;
}
