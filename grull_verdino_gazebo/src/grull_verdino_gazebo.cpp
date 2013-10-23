#include <ros/ros.h>

#include<sensor_msgs/JointState.h>
#include<nav_msgs/Odometry.h>

#include <grull_ackermann_msgs/bc_alarmas.h>
#include <grull_ackermann_msgs/bc_estado.h>
#include <grull_ackermann_msgs/bc_PID_Avance.h>
#include <grull_ackermann_msgs/bc_comando_directo.h>
#include <grull_ackermann_msgs/bc_consigna_velocidad.h>
#include <grull_ackermann_msgs/bc_consigna_volante.h>
#include <grull_ackermann_msgs/bc_stop_control_vel.h>
#include <grull_ackermann_msgs/bc_set_automatico.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

//#include <grull_verdino_base_controller/VerdinoOdometry.h>

/** Nombre de los tópicos */
#define TOPICO_CONSIGNA_VEL  "verdino/consignaVel"
#define TOPICO_CONSIGNA_VOL  "verdino/consignaVolante"
#define TOPICO_STOP_CTRL_VEL  "verdino/stopControlVel"
#define TOPICO_SET_AUTOMATICO  "verdino/setAutomatico"
#define TOPICO_COMANDO_DIRECTO  "verdino/comandoDirecto"
#define TOPICO_ODOMETRIA  "/odom"
#define TOPICO_ALARMAS  "verdino/alarmas"
#define TOPICO_ESTADO  "verdino/estado"
#define TOPICO_PID_AVANCE  "verdino/pidavance"

#define TOPICO_ACKERMANN_STATE  "ackermann_vehicle/joint_states"
#define TOPICO_ACKERMANN_ODOMETRY  "ackermann_vehicle/odom"
#define TOPICO_ACKERMANN_CMD  "ackermann_vehicle/ackermann_cmd"

class AckermannController {
    public:
    AckermannController();
    void callbackConsignaVolante(const grull_ackermann_msgs::bc_consigna_volante::ConstPtr& msgIn);
    void callbackConsignaVel(const grull_ackermann_msgs::bc_consigna_velocidad::ConstPtr& msgIn);
    void callbackSteeringStatus(const sensor_msgs::JointState::ConstPtr& msgIn);
    void callbackOdometry(const nav_msgs::Odometry::ConstPtr& msgIn);

    ackermann_msgs::AckermannDriveStamped msg;
    grull_ackermann_msgs::bc_estado vehicleStatus;
    nav_msgs::Odometry odomMsg;
};

AckermannController::AckermannController(){
    msg.header.seq = 0;
    msg.drive.steering_angle = 0;
    msg.drive.steering_angle_velocity = 0;
    msg.drive.speed = 0;
    msg.drive.acceleration = 0;
    msg.drive.jerk = 0;


    vehicleStatus.header.seq = 0;
    odomMsg.header.seq = 0;

}

void AckermannController::callbackConsignaVel(const grull_ackermann_msgs::bc_consigna_velocidad::ConstPtr& msgIn){
    msg.drive.speed = (float)msgIn->consignaVelocidad;
    msg.header.stamp = ros::Time::now();
    msg.header.seq++;
}


void AckermannController::callbackConsignaVolante(const grull_ackermann_msgs::bc_consigna_volante::ConstPtr& msgIn){
    msg.drive.steering_angle = (float)msgIn->consignaVolante;
}


void AckermannController::callbackSteeringStatus(const sensor_msgs::JointState::ConstPtr& msgIn){
    //Left and Right Wheel Steering Position
    vehicleStatus.anguloVolante = (msgIn->position[4] + msgIn->position[9])/2;
    vehicleStatus.header.stamp = ros::Time::now();
    vehicleStatus.header.seq++;

}


void AckermannController::callbackOdometry(const nav_msgs::Odometry::ConstPtr& msgIn){
    vehicleStatus.velocidad = msgIn->twist.twist.linear.x;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "grull_verdino_gazebo");
    AckermannController ackermannController;
    ros::NodeHandle nh;
    //SUBSCRIBERS
    ros::Subscriber conVelSub = nh.subscribe<grull_ackermann_msgs::bc_consigna_velocidad>(TOPICO_CONSIGNA_VEL,10,
            &AckermannController::callbackConsignaVel, &ackermannController);
    ros::Subscriber conVolSub = nh.subscribe<grull_ackermann_msgs::bc_consigna_volante>(TOPICO_CONSIGNA_VOL, 10,
            &AckermannController::callbackConsignaVolante, &ackermannController);
    ros::Subscriber statVehSub= nh.subscribe<sensor_msgs::JointState>(TOPICO_ACKERMANN_STATE , 10,
            &AckermannController::callbackSteeringStatus, &ackermannController);
    ros::Subscriber odomSub= nh.subscribe<nav_msgs::Odometry>(TOPICO_ACKERMANN_ODOMETRY , 10,
            &AckermannController::callbackOdometry, &ackermannController);
    //PUBLISHERS
    ros::Publisher ackPub = nh.advertise<ackermann_msgs::AckermannDriveStamped>(TOPICO_ACKERMANN_CMD, 1000);
    ros::Publisher statusPub = nh.advertise<grull_ackermann_msgs::bc_estado>(TOPICO_ESTADO, 1000);

    int seqVel = 0;
    int seqStatus = 0;
    while (ros::ok()){
        if(seqVel < ackermannController.msg.header.seq){
            ackPub.publish(ackermannController.msg);
            seqVel++;
        }
        if(seqStatus < ackermannController.vehicleStatus.header.seq){
            statusPub.publish(ackermannController.vehicleStatus);
            seqStatus++;
        }
        ros::spinOnce();
    }
    return 0;
}
