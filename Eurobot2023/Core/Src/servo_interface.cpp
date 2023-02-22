/*
 * servo_driver.cpp
 *
 *  Created on: Jan 28, 2023
 *      Author: Valery_Danilov
 *      Edited: Maxim_Popov
 */

#include "servo_interface.h"


void Servo_Interface::Init(I2C_HandleTypeDef *hi2c){
        PCA9685_Init(hi2c);
        _node.subscribe(_servo_cmd_subscriber);
}


Servo_Interface::Servo_Interface(
        std::initializer_list<Servo> servo_params_list,
        I2C_HandleTypeDef *hi2c,
        ros::NodeHandle& node,
        //const char* servo_state_topic_name,
        const char* servo_cmd_topic
        ):
        //_servo_state_publisher(servo_state_topic_name, &_cur_angle),
        _node(node),
        _servo_cmd_subscriber(servo_cmd_topic, [&, this](const std_msgs::Float32MultiArray& msg){this->_write(msg);}),
        _servos(servo_params_list)
    {
        //Init(hi2c);
        //_servos = *servos;
    }
void Servo_Interface::_write(const std_msgs::Float32MultiArray& msg){
    uint8_t channel = 0;
    uint16_t value = 0;
    float angle = 0;

    if (_servos.size() != 0 && msg.layout.dim->size == _servos.size()){  //are servo's exist's and len of data is correct

        for (uint8_t i = 0; i < msg.layout.dim->size; i++) {      //check angle

            channel = i;
            angle = msg.data[i];
           static float min_angle = _servos[i].get_min_angle();
           static float max_angle = _servos[i].get_max_angle();

            if( msg.data[i] < min_angle ){
                angle = min_angle;
            }
            if( msg.data[i] > max_angle ){
                angle = max_angle;
            }

            value = (angle - min_angle) * ((float)SERVO_MAX - (float)SERVO_MIN) / (max_angle - min_angle) + (float)SERVO_MIN;  //TODO SERVO MAX, SERVO MIN ?
            PCA9685_SetPin(channel, value, 0);    //write on PCA9685
            ///PCA9685_SetPin(Channel, Value, Invert)
        }

    }
}
