#include "Motor.h"

Motor::Motor(uint32_t p_MotorId){
    m_MotorId = p_MotorId;
}

/******************************************************/
void Motor::enterMode(CanFrame* msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFC;
    msg->identifier = m_MotorId;
    msg->extd = 0;
    msg->ss = 1;
    msg->data_length_code = 8;
}

/******************************************************/
void Motor::exitMode(CanFrame* msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFD;
    msg->identifier = m_MotorId;
    msg->extd = 1;
    msg->ss = 1;
    msg->data_length_code = 8;
}

/******************************************************/
void Motor::zeroSet(CanFrame* msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFE;
    msg->identifier = m_MotorId;
    msg->extd = 0;
    msg->ss = 1;
    msg->data_length_code = 8;
    
}

/******************************************************/
void Motor::zero(CanFrame* msg){
    msg->data[0] = 0xFF;
    msg->data[1] = 0xFF;
    msg->data[2] = 0xFF;
    msg->data[3] = 0xFF;
    msg->data[4] = 0xFF;
    msg->data[5] = 0xFF;
    msg->data[6] = 0xFF;
    msg->data[7] = 0xFF;
    msg->identifier = m_MotorId;
    msg->extd = 0;
    msg->ss = 1;
    msg->data_length_code = 8;
}

/******************************************************/
void Motor::pack_cmd(CanFrame* msg, float p_des, float v_des, float kp, float kd, float t_ff){
    ///CAN Command Packet Structure (64 bits total, 8 bytes)///
    /// Bits  0-15:  Position command, range: -4*pi to 4*pi
    /// Bits 16-27:  Velocity command, range: -30 to +30 rad/s
    /// Bits 28-39:  Kp (Proportional gain), range: 0 to 500 N-m/rad
    /// Bits 40-51:  Kd (Derivative gain), range: 0 to 100 N-m*s/rad
    /// Bits 52-63:  Feed forward torque, range: -18 to 18 N-m
    
    ///limit data to be withing bounds///
    p_des = constrain(p_des, P_MIN, P_MAX); ///fminf(fmaxf(P_MIN, p_in(, P_MAX);
    v_des = constrain(v_des, V_MIN, V_MAX); ///fminf(fmaxf(V_MIN, v_in(, V_MAX);
    kp = constrain(kp, KP_MIN, KP_MAX); ///fminf(fmaxf(KP_MIN, kp_in(, KP_MAX);
    kd = constrain(kd, KD_MIN, KD_MAX); ///fminf(fmaxf(KD_MIN, kd_in(, KD_MAX);
    t_ff = constrain(t_ff, T_MIN, T_MAX); ///fminf(fmaxf(T_MIN, t_in(, V_MAX);


    ///convert floats to unsigned ints///
    unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);


    /// pack ints into the can buffer///
    byte buf[8];
    //buf[0] = p_int >> 8;
    //buf[1] = p_int & 0xFF;
    //buf[2] = v_int >> 4;
    //buf[3] = ((v_int & 0xF) <<4) | (kp_int >>8);
    //buf[4] = kp_int & 0xFF;
    //buf[5] = kd_int >>4;
   // buf[6] = ((kd_int & 0xF) <<4) | (t_int >>8);
    //buf[7] = t_int & 0xFF;
    buf[0] = kp_int>>4; //KP high 8 bits
    buf[1] = ((kp_int&0xF)<<4)|( kd_int>>8); //KP Low 4 bits, Kd High 4 bits
    buf[2] = kd_int&0xFF; //Kd low 8 bits
    buf[3] = p_int>>8; //position high 8 bits
    buf[4] = p_int&0xFF; //position low 8 bits
    buf[5] = v_int>>4; //speed high 8 bits
    buf[6] = ((v_int&0xF)<<4)|(t_int>>8); //speed low 4 bits torque high 4 bits
    buf[7] = t_int&0xff; //torque low 8 bits
    for(int i = 0;i < 8; i++){
        msg->data[i] = buf[i];
    }
    msg->identifier = m_MotorId;
    msg->extd = 1;
    msg->ss = 1;
    msg->data_length_code = 8;
}

/******************************************************/
void Motor::unpack_reply(CanFrame* msg) {
///CAN Reply Packet Structure (40 bits total, 5 bytes)///
/// Bits  0-15:  Position, range: -4*pi to 4*pi
/// Bits 16-27:  Velocity, range: -30 to +30 rad/s
/// Bits 28-39:  Current, range: -40 to 40 A

    //read data
    //MCP_CAN::readMsgBuf(*len,  buf[])

    
    /// unpack ints from CAN buffer ///
    unsigned int id = msg->data[0];
    unsigned int p_int = (msg->data[1] << 8) | msg->data[2];
    unsigned int v_int = (msg->data[3] << 4) | (msg->data[4] >> 4);
    unsigned int i_int = ((msg->data[4] & 0xF) << 8) | msg->data[5];
    unsigned int temp = msg->data[6]-40;
    /// convert uints to floats ///
    Serial.println(id);
    Serial.println(temp);
    Serial.println(p_int);

}

/******************************************************/
unsigned int Motor::float_to_uint(float x, float x_min, float x_max, int bits){
    ///Converts a  float to an unsigned int, given range and number of bits///
    float span = x_max-x_min;
    float offset = x_min;
    unsigned int pgg = 0;
    if(bits==12){
        pgg = (unsigned int) ((x-offset)*4095.0/span);
    }
    if(bits==16){
        pgg = (unsigned int) ((x-offset)*65535.0/span);
    }
    return pgg;
}

/******************************************************/
float Motor::uint_to_float(unsigned int x_int, float x_min, float x_max, int bits){
    ///converts unsigned int to float, given range and number of bits///
    float span = x_max-x_min;
    float offset = x_min;
    float pgg = 0;
    if (bits==12){
        pgg = ((float) x_int)*span/4095 + offset;
    }
    if (bits==16){
        pgg = ((float) x_int)*span/65535.0 + offset;
    }
    return pgg;
}

