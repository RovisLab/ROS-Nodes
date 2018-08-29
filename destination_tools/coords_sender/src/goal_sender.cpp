#include <ros/ros.h>
#include <coords_msgs/Coords.h>
#include < <<tip_mesaj,gen cel de sus>>>

void nume_functie(const <<nume_tip de mesaj>>& msg)
{
    //verifici datele din mesaj.
    //datele de verificat sunt: label-ul mesajului(denumirea obiectului detectat).
    //                          probabilitatea
    //de trimis catre topicul nostru, x si z din mesajul acesta.
    coords_msgs::Coords coordonate;
    coordonate.goal_x = msg.x;
    coordonate.goal_y = msg.y;//or z
    ROS_INFO("se transmit coordonatele %f, %f",coordonate.goal_x,coordonate.goal_y);
    pub.publish(coordonate);
    //also.ar fi fain sa nu verifice de fiecare data daca se detecteaza obiectul, ci doar cand dorim noi.
    //chestia asta o putem face din main. Sau pur si simplu daca s-a trimis un mesaj, sa nu mai urmeze si altele.cauta info despre ros::spin, pt asta.


}
// create a subscriber function
//inside it check for the info and the create a mesage to send to the topic.
int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_sender");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("numele topicului",10,<<numele_functiei>>);
    ros::Publisher pub = nh.advertise<coords_msgs::Coords>("coord_topic", 10,true);

    ros::spin();

    return 0;
}



