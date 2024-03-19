#include <stdio.h>
#include<driver/i2c.h>
#include<math.h>



/*欧拉角copy的*/
 
 
float accel_scale=16384;
float gyro_scale=131;
float calibration_array[6]={0};
 
void write_register(uint8_t *data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x68<<1|0, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
 
 
uint8_t read_register(uint8_t reg){
    uint8_t data;
    i2c_master_write_read_device(0,0x68,&reg,1,&data,1,1000/portTICK_PERIOD_MS);
    return data;
}
 
 
void I2C_init(void){
    i2c_config_t conf={
        .mode=I2C_MODE_MASTER,
        .sda_io_num=4,
        .scl_io_num=5,
        .sda_pullup_en=1,
        .scl_pullup_en=1,
        .master.clk_speed=400000
    };
    i2c_param_config(0,&conf);
    i2c_driver_install(0,I2C_MODE_MASTER,0,0,0);
}
 
 
void MPU6050_init(void){    
    uint8_t gyroscope_config_register=27;
    uint8_t accel_config_register=28;
    uint8_t poweron[]={0x6B,0x01};
    uint8_t poweron2[]={0x6C,0x00};
    uint8_t DLPF_CFG[]={0x1A,0x06};
    uint8_t GYRO_CONFIG[]={0x1B,0x18};
    uint8_t ACCEL_CONFIG[]={0x1C,0x18};
    uint8_t SMPLRT_DIV[]={0x19,0x09};
 
    write_register(poweron);
    write_register(poweron2);
    write_register(DLPF_CFG);
    write_register(GYRO_CONFIG);
    write_register(ACCEL_CONFIG);
    write_register(SMPLRT_DIV);
    
   
}
 
 
struct AccelGyroData_t
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
};
struct Angle_t{
    float pitch;
    float row;
    float yaw;
};
 
struct AccelGyroData_t get_raw_mpu6050_data(){
    uint8_t accelGyroRegArray[]={59,60,61,62,63,64,67,68,69,70,71,72};
    float AccelGyroDataArray[6];
    for(int i=0;i<12;i+=2){
        AccelGyroDataArray[i/2]=(read_register(accelGyroRegArray[i])<<8)+read_register(accelGyroRegArray[i+1]);
        if(i<6){
            if(AccelGyroDataArray[i/2]>32767) AccelGyroDataArray[i/2]=AccelGyroDataArray[i/2]/accel_scale-4;
            else AccelGyroDataArray[i/2]=AccelGyroDataArray[i/2]/accel_scale;
        }
        else{
            if(AccelGyroDataArray[i/2]>32767) AccelGyroDataArray[i/2]=AccelGyroDataArray[i/2]/gyro_scale-500;
            else AccelGyroDataArray[i/2]=AccelGyroDataArray[i/2]/gyro_scale;
        };
    };
    struct AccelGyroData_t AccelGyroData={AccelGyroDataArray[0],AccelGyroDataArray[1],AccelGyroDataArray[2],AccelGyroDataArray[3],AccelGyroDataArray[4],AccelGyroDataArray[5]};
    return AccelGyroData;
}
 
 
void get_mpu6050_calibration(float calibration_array[]){
    struct AccelGyroData_t accelGyroData;
    for(int i=0;i<20;i++){
        accelGyroData=get_raw_mpu6050_data();
        calibration_array[0]+=accelGyroData.accelX;
        calibration_array[1]+=accelGyroData.accelY;
        calibration_array[2]+=accelGyroData.accelZ;
        calibration_array[3]+=accelGyroData.gyroX;
        calibration_array[4]+=accelGyroData.gyroY;
        calibration_array[5]+=accelGyroData.gyroZ;
        vTaskDelay(5/portTICK_PERIOD_MS);
    };
    for(int i=0;i<6;i++){
        calibration_array[i]/=20;
        if(i==2) calibration_array[i]=1-calibration_array[i];
        else calibration_array[i]=0-calibration_array[i];
    };
    printf("calibration succees\n");
}
 
 
struct AccelGyroData_t get_mpu6050_data(float calibration_array[]){
    struct AccelGyroData_t accelGyroData=get_raw_mpu6050_data();
    accelGyroData.accelX+=calibration_array[0];
    accelGyroData.accelY+=calibration_array[1];
    accelGyroData.accelZ+=calibration_array[2];
    accelGyroData.gyroX+=calibration_array[3];
    accelGyroData.gyroY+=calibration_array[4];
    accelGyroData.gyroZ+=calibration_array[5];
    return accelGyroData;
}
 
struct Angle_t get_angle(struct Angle_t angle_data)
{
    uint8_t SAMPLE_DIV=25;
    float k=0.95;
    struct AccelGyroData_t accelGyroData=get_mpu6050_data(calibration_array);
    // float sample_rate=(1+read_register(SAMPLE_DIV))/8000;
    float accel_angle_pitch,accel_angle_row,gyro_angle_pitch,gyro_angle_row,gyro_angle_yaw;
 
    // 加速度算出的pitch，row
    accel_angle_pitch=atan(accelGyroData.accelX/accelGyroData.accelZ)*57.3;
    accel_angle_row=atan(accelGyroData.accelY/accelGyroData.accelZ)*57.3;
 
    // 陀螺仪算出的pitch，row，yaw
    gyro_angle_row=accelGyroData.gyroX/7500;
    gyro_angle_pitch=accelGyroData.gyroY/7500;
    gyro_angle_yaw=accelGyroData.gyroZ/7500;
 
    // 加权计算得出欧拉角
    angle_data.pitch=k*(accel_angle_pitch+gyro_angle_pitch)+(1-k)*angle_data.pitch;
    angle_data.row=k*(accel_angle_row+gyro_angle_row)+(1-k)*angle_data.row;
    angle_data.yaw=0.9*(angle_data.yaw+gyro_angle_yaw)+0.1*angle_data.yaw;
 
    return angle_data;
}
 
 
 
void app_main(void)
{
    I2C_init();
    MPU6050_init();
    
    get_mpu6050_calibration(calibration_array);
    struct AccelGyroData_t data;
    struct Angle_t angle={0,0,0};
    while(1){
        struct AccelGyroData_t accelGyroData=get_raw_mpu6050_data(calibration_array);
        angle=get_angle(angle);
        printf("%f %f %f\n",angle.pitch,angle.row,angle.yaw);
        printf("%f,%f,%f\n",accelGyroData.accelX,accelGyroData.accelY,accelGyroData.accelZ);
        printf("%f,%f,%f\n",accelGyroData.gyroX,accelGyroData.gyroY,accelGyroData.gyroZ);
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
    
 
    
 