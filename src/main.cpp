#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <diagnostic_msgs/msg/diagnostic_array.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/empty.h>

#include "pcap04.h"
#include "Adafruit_BMP3XX.h"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

#include "Adafruit_SPIDevice.h"

#include "config.h"
#include "diagnostics.h"

rcl_publisher_t p14_1_humi_publisher;
rcl_publisher_t p14_1_temp_publisher;
rcl_publisher_t p14_2_humi_publisher;
rcl_publisher_t p14_2_temp_publisher;
rcl_publisher_t pressure_publisher;
rcl_publisher_t bmp390_temp_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_publisher_t diagnostics_publisher;

rcl_service_t led_service;
rcl_service_t aquisition_service;
rcl_service_t calibrate_IMU_service;

sensor_msgs__msg__RelativeHumidity p14_1_humi;
sensor_msgs__msg__RelativeHumidity p14_2_humi;
sensor_msgs__msg__Temperature p14_1_temp;
sensor_msgs__msg__Temperature p14_2_temp;
sensor_msgs__msg__Temperature bmp390_temp;
sensor_msgs__msg__FluidPressure pressure_data;
sensor_msgs__msg__Imu imu_data;
sensor_msgs__msg__MagneticField compass_data;
diagnostic_msgs__msg__DiagnosticStatus * metsensor_status;
diagnostic_msgs__msg__KeyValue * led_keyval;
diagnostic_msgs__msg__KeyValue * aquire_keyval;
diagnostic_msgs__msg__KeyValue * imu_keyval;
diagnostic_msgs__msg__KeyValue__Sequence teensy_key_array;
diagnostic_msgs__msg__DiagnosticStatus__Sequence status_array;
diagnostic_msgs__msg__DiagnosticArray * dia_array;
std_srvs__srv__SetBool_Request led_request;
std_srvs__srv__SetBool_Response led_response;
std_srvs__srv__SetBool_Request aquire_request;
std_srvs__srv__SetBool_Response aquire_response;
std_srvs__srv__Empty_Request imu_request;
std_srvs__srv__Empty_Response imu_response;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t diagnostic_timer;
rcl_timer_t sensor_timer;

mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
mpud::float_axes_t accelG;   // accel axes in (g) gravity format
mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {coms_error(LED_PIN);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {coms_error(LED_PIN);}}

static volatile uint32_t epoc_time_nanos;

void init_hw_peripherals(){

  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(PCAP1_CS_PIN, OUTPUT);              // PCAP1 serial select pin
  digitalWrite(PCAP1_CS_PIN, HIGH);           // PCAP1 serial de-select

  pinMode(PCAP2_CS_PIN, OUTPUT);              // PCAP2 serial select pin
  digitalWrite(PCAP2_CS_PIN, HIGH);           // PCAP2 serial de-select
  
  pinMode(MPU_CS_PIN, OUTPUT);                // MPU6500 serial select pin
  digitalWrite(MPU_CS_PIN, HIGH);             // MPU6500 serial de-select     
  
  pinMode(HEATER1_SEL_PIN, OUTPUT);       
  digitalWrite(HEATER1_SEL_PIN, LOW);       

  pinMode(HEATER2_SEL_PIN, OUTPUT);       
  digitalWrite(HEATER2_SEL_PIN, LOW);     

  SPI.begin();

  if (!bmp390.begin_I2C(BMP390_I2C_ADDR)) {
    LOG_ERROR("Could not find a valid BMP390, check wiring!");
    coms_error(LED_PIN);
  }
  bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp390.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp390.setOutputDataRate(BMP3_ODR_50_HZ); 

  while (uint8_t err = mpu.testConnection()){
    LOG_ERROR("Could not find a valid MPU6500, check wiring!");
    coms_error(LED_PIN);
  }

  mpu.initialize();

  delay(1000);

  };

void publish_diagnostics() {
  // update key value array
  teensy_key_array.data[0] = *led_keyval;
  teensy_key_array.data[1] = *aquire_keyval;
  teensy_key_array.data[2] = *imu_keyval;
  metsensor_status->values = teensy_key_array;

  // update status array
  status_array.data[0] = *metsensor_status;
  dia_array->status = status_array;

  RCSOFTCHECK(rcl_publish(&diagnostics_publisher, dia_array, NULL));
}

void led_service_callback(const void * request, void * response){
  std_srvs__srv__SetBool_Request * req_in =
    (std_srvs__srv__SetBool_Request *) request;
  std_srvs__srv__SetBool_Response * res_in =
    (std_srvs__srv__SetBool_Response *) response;

  if (req_in->data) {
    // FIXME: turn on led here

    // send response
    const char *message = "ON - recieved, but not used";
    int msg_length = strlen(message) + 1;
    res_in->message.data = (char*)malloc(msg_length*sizeof(char));
    res_in->message.capacity = msg_length;
    snprintf(res_in->message.data, res_in->message.capacity, message);
    res_in->message.size = strlen(res_in->message.data);
    res_in->success = false;  // FIXME: update when command executed
    led_keyval = update_diagnostic_KeyValue(led_keyval, "ON");
  } else {
    // FIXME: turn off led here
    //Send response
    const char *message = "OFF - recieved, but not used";
    int msg_length = strlen(message) + 1;
    res_in->message.data = (char*)malloc(msg_length*sizeof(char));
    res_in->message.capacity = msg_length;
    snprintf(res_in->message.data, res_in->message.capacity, message);
    res_in->message.size = strlen(res_in->message.data);
    res_in->success = false;  // FIXME: update when command executed
    led_keyval = update_diagnostic_KeyValue(led_keyval, "OFF");
    }
}

void aquisition_service_callback(const void * request, void * response){
  std_srvs__srv__SetBool_Request * req_in =
    (std_srvs__srv__SetBool_Request *) request;
  std_srvs__srv__SetBool_Response * res_in =
    (std_srvs__srv__SetBool_Response *) response;

  if (req_in->data) {
    // FIXME: Start polling and publishing here

    // send response
    const char *message = "ON - recieved, but not used";
    int msg_length = strlen(message) + 1;
    res_in->message.data = (char*)malloc(msg_length*sizeof(char));
    res_in->message.capacity = msg_length;
    snprintf(res_in->message.data, res_in->message.capacity, message);
    res_in->message.size = strlen(res_in->message.data);
    res_in->success = false;  // FIXME: update when command executed
    aquire_keyval = update_diagnostic_KeyValue(aquire_keyval, "ON");
  } else {
    // FIXME: Start polling and publishing here

    //Send response
    const char *message = "OFF - recieved, but not used";
    int msg_length = strlen(message) + 1;
    res_in->message.data = (char*)malloc(msg_length*sizeof(char));
    res_in->message.capacity = msg_length;
    snprintf(res_in->message.data, res_in->message.capacity, message);
    res_in->message.size = strlen(res_in->message.data);
    res_in->success = false;  // FIXME: update when command executed
    aquire_keyval = update_diagnostic_KeyValue(aquire_keyval, "OFF");
    }
}

void imu_service_callback(const void * request, void * response){
      // FIXME: Calibrate IMU here
      imu_keyval = update_diagnostic_KeyValue(imu_keyval, "Calibrating");
}

// Timer callback which publishes diaganostic status message at set interval
void diagnostic_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    publish_diagnostics();
  }
}

// Timer callback which publishes values at set interval
void sensor_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // FIXME: publish stuff here

    pressure_data.header.stamp.nanosec = epoc_time_nanos;
    pressure_data.fluid_pressure = bmp390.readPressure();
    bmp390_temp.temperature = bmp390.readTemperature();
    
    RCSOFTCHECK(rcl_publish(&bmp390_temp_publisher, &bmp390_temp, NULL));
    RCSOFTCHECK(rcl_publish(&pressure_publisher, &pressure_data, NULL));

  }
}



/**
* @brief Fills out the diagnostic message structure with the default values
*/
void init_diagnostics() {
  // Teensy Status
  metsensor_status = create_diagnostic_status(metsensor_status,
                    "Metsensor",
                    "Awaiting Setup",
                    metsensor_id,
                    diagnostic_msgs__msg__DiagnosticStatus__OK);
  // Teensy Status Key-value pairs
  led_keyval = create_diagnostic_KeyValue(led_keyval, "LED", "Init");
  aquire_keyval = create_diagnostic_KeyValue(aquire_keyval, "Aquiring", "Init");
  imu_keyval = create_diagnostic_KeyValue(imu_keyval, "IMU", "Init");

  // add pairs to status in array
  diagnostic_msgs__msg__KeyValue__Sequence__init(&teensy_key_array, 3);
  teensy_key_array.data[0] = *led_keyval;
  teensy_key_array.data[1] = *aquire_keyval;
  teensy_key_array.data[2] = *imu_keyval;
  metsensor_status->values = teensy_key_array;

  // Fill diagnostic array with statuses
  diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&status_array, 1);
  dia_array = diagnostic_msgs__msg__DiagnosticArray__create();
  status_array.data[0] = *metsensor_status;
  dia_array->status = status_array;
}

void setup() {
  //set_microros_transports();
  
  init_hw_peripherals();
  
  pcap1.init_nvram();

  PRINTLN("current config");
  pcap1.print_config();

  metsensor_pcap_config = pcap1.get_config();

  metsensor_pcap_config.C_TRIG_SEL = 0x02;
  metsensor_pcap_config.C_DIFFERENTIAL = 0x00;
  metsensor_pcap_config.C_COMP_EXT = 0x01;
  metsensor_pcap_config.C_FLOATING = 0x01;
  metsensor_pcap_config.C_REF_INT = 0x00;
  metsensor_pcap_config.C_COMP_INT = 0b1;
  metsensor_pcap_config.C_COMP_EXT = 0b1;
  metsensor_pcap_config.C_FAKE = 0x00;
  metsensor_pcap_config.C_AVRG = 0x04;
  metsensor_pcap_config.CONV_TIME = 0x2710;
  metsensor_pcap_config.PRECHARGE_TIME = 0x05;
  metsensor_pcap_config.FULLCHARGE_TIME =  0x05;  
  metsensor_pcap_config.DISCHARGE_TIME = 0x10;
  metsensor_pcap_config.C_PORT_EN = 0x3F;
  metsensor_pcap_config.R_TRIG_SEL = 0x05;
  metsensor_pcap_config.R_TRIG_PREDIV = 0x01;
  metsensor_pcap_config.R_AVRG = 0x01;
  metsensor_pcap_config.R_FAKE = 0x00;
  metsensor_pcap_config.R_PORT_EN_IMES = 0b1;
  metsensor_pcap_config.R_PORT_EN_IREF = 0b1;
  metsensor_pcap_config.R_PORT_EN = 0b10;
  metsensor_pcap_config.RDCHG_INT_EN = 0x01;
  metsensor_pcap_config.RDCHG_INT_SEL0 = 0x01;
  metsensor_pcap_config.RDCHG_INT_SEL1 = 0x01;
  metsensor_pcap_config.RCHG_SEL = 0x00;
  
  pcap1.update_config(metsensor_pcap_config);

  PRINTLN("updated config");
  pcap1.print_config();
  
  pcap1.init_slave();

  delay(1);

  // pcap1.send_command(CDC_START);

  delay(1);
  
  // allocator = rcl_get_default_allocator();

  // // create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // delay(1000);

  // // create node
  // node = rcl_get_zero_initialized_node();
  // rcl_node_options_t node_ops = rcl_node_get_default_options();
  // node_ops.domain_id = ros_domain_id;
  // RCCHECK(rclc_node_init_with_options(&node, "teensy_node", "", &support, &node_ops));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &p14_1_humi_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
  //     "p14_1/humidity"));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &p14_1_temp_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
  //     "p14_1/temp"));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &p14_2_humi_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
  //     "p14_2/humidity"));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &p14_2_temp_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
  //     "p14_2/temp"));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &pressure_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
  //     "BMP390/pressure" ));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &bmp390_temp_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
  //     "BMP390/temp" ));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &imu_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //   "imu" ));
  // delay(1000);

  // // create publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &mag_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
  //   "magnetometer" ));
  // delay(1000);

  // // create Diagnostic Status publisher
  // RCCHECK(rclc_publisher_init_best_effort(
  //   &diagnostics_publisher,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray),
  //   "diagnostics"));
  // delay(1000);

  // // // create service
  // // RCCHECK(rclc_service_init_default(
  // //   &led_service,
  // //   &node,
  // //   ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
  // //   "nav_led"));
  // // delay(1000);

  // // create service
  // RCCHECK(rclc_service_init_default(
  //   &aquisition_service,
  //   &node,
  //   ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
  //   "aquire_data"));
  // delay(1000);

  // // // create service
  // // RCCHECK(rclc_service_init_default(
  // //   &calibrate_IMU_service,
  // //   &node,
  // //   ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Empty),
  // //   "calibrate_imu"));
  // // delay(1000);

  // // create timer, to pub diagnostics at xhz
  // diagnostic_timer = rcl_get_zero_initialized_timer();
  // RCCHECK(rclc_timer_init_default(
  //   &diagnostic_timer,
  //   &support,
  //   RCL_MS_TO_NS(1000/diagnostic_frequency_hz),  // convert Hz to ms
  //   diagnostic_timer_callback));
  // delay(1000);

  // // create timer, to pub sensors at xhz
  // sensor_timer = rcl_get_zero_initialized_timer();
  // RCCHECK(rclc_timer_init_default(
  //   &sensor_timer,
  //   &support,
  //   RCL_MS_TO_NS(1000/sensor_frequency_hz),  // convert Hz to ms
  //   sensor_timer_callback));

  // // create executor
  // // total number of handles = #subscriptions + #timers + #services + #clients
  // unsigned int num_handles = 0 + 1 + 1 + 0;
  // executor = rclc_executor_get_zero_initialized_executor();
  // RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  // // RCCHECK(rclc_executor_add_service(&executor, &led_service, &led_request,
  // //                                   &led_response, led_service_callback));
  // RCCHECK(rclc_executor_add_service(&executor, &aquisition_service, &aquire_request,
  //                                   &aquire_response, aquisition_service_callback));
  // // RCCHECK(rclc_executor_add_service(&executor, &calibrate_IMU_service, &imu_request,
  // //                                   &imu_response, imu_service_callback));
  // RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
  // // RCCHECK(rclc_executor_add_timer(&executor, &diagnostic_timer));

  // init_diagnostics();


}  // end setup

void loop() {
  
  mpu.acceleration(&accelRaw);  // fetch raw data from the registers
  mpu.rotation(&gyroRaw);       // fetch raw data from the registers
  // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
  // Convert
  accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
  gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
  
  // Debug
  MPU_LOGD("accelRaw: [", accelRaw.x, accelRaw.y, accelRaw.z, "] \t");
  MPU_LOGD("accel: [", DebugLogPrecision::FOUR, accelG.x, DebugLogPrecision::FOUR, accelG.y, DebugLogPrecision::FOUR, accelG.z, "] (G) \t");
  MPU_LOGD("gyro : [", DebugLogPrecision::FOUR, gyroDPS[0], DebugLogPrecision::FOUR, gyroDPS[1], DebugLogPrecision::FOUR, gyroDPS[2], "] (º/s)\n");

/*   
  pcap_status = pcap1.get_status(false);
  while(pcap_status.CDC_ACTIVE || pcap_status.COMB_ERR){
    if (pcap_status.COMB_ERR){
      Serial.print("-COMB_ERR");
      if (pcap_status.ERR_OVERFL){
        Serial.print("--ERR_OVERFL");
      }
      if (pcap_status.MUP_ERR){
        Serial.print("--MUP_ERR");
      }
      if (pcap_status.RDC_ERR){
        Serial.print("--RDC_ERR");
      }
      if(pcap_status.C_PORT_ERR0){
        Serial.print("--C_PORT_ERR0");
      }
      if(pcap_status.C_PORT_ERR1){
        Serial.print("--C_PORT_ERR1");
      }
      if(pcap_status.C_PORT_ERR2){
        Serial.print("--C_PORT_ERR2");
      }
      if(pcap_status.C_PORT_ERR3){
        Serial.print("--C_PORT_ERR3");
      }
      if(pcap_status.C_PORT_ERR4){
        Serial.print("--C_PORT_ERR4");
      }
      if(pcap_status.C_PORT_ERR5){
        Serial.print("--C_PORT_ERR5");
      }
      if(pcap_status.C_PORT_ERR_INT){
        Serial.print("--C_PORT_ERR_INT");
      }                          

    }
    if(pcap_status.POR_FLAG_CONFIG){
      Serial.print("POR_FLAG_CONFIG");      
    }
    if (pcap_status.POR_CDC_DSP_COLL){
      Serial.print("POR_CDC_DSP_COLL");      
    }
    if (pcap_status.POR_FLAG_WDOG){
      Serial.print("POR_FLAG_WDOG");      
    }
    if (pcap_status.POR_FLAG_WDOG){
      Serial.print("POR_FLAG_WDOG");      
    }        
    pcap_status = pcap1.get_status(false);
    // Serial.print(" * ");
    delay(10);
  }

  pcap_status = pcap1.get_status(false); 
  pcap_results = pcap1.get_results();
  current_micros = micros();

  results_json["results0_f"] = pcap_results.C0_over_CREF;
  results_json["results1_f"] = pcap_results.C1_over_CREF;
  results_json["results2_f"] = pcap_results.C2_over_CREF;
  results_json["results3_f"] = pcap_results.C3_over_CREF;
  results_json["results4_f"] = pcap_results.C4_over_CREF;
  results_json["results5_f"] = pcap_results.C5_over_CREF;
  results_json["results6_f"] = pcap_results.PT1_over_PTREF;        
  results_json["results7_f"] = pcap_results.PTInternal_over_PTREF;
  results_json["time"] = current_micros;
  results_json["RUNBIT"] = pcap_status.RUNBIT;
  results_json["CDC_ACTIVE"] = pcap_status.CDC_ACTIVE;
  results_json["RDC_READY"] = pcap_status.RDC_READY;
  results_json["AUTOBOOT_BUSY"] = pcap_status.AUTOBOOT_BUSY;
  results_json["POR_CDC_DSP_COLL"] = pcap_status.POR_CDC_DSP_COLL;
  results_json["POR_FLAG_WDOG"] = pcap_status.POR_FLAG_WDOG;
  results_json["COMB_ERR"] = pcap_status.COMB_ERR;
  results_json["ERR_OVERFL"] = pcap_status.ERR_OVERFL;
  results_json["MUP_ERR"] = pcap_status.MUP_ERR;
  results_json["RDC_ERR"] = pcap_status.RDC_ERR;
  results_json["C_PORT_ERR0"] = pcap_status.C_PORT_ERR0;
  results_json["C_PORT_ERR1"] = pcap_status.C_PORT_ERR1;
  results_json["C_PORT_ERR2"] = pcap_status.C_PORT_ERR2;
  results_json["C_PORT_ERR3"] = pcap_status.C_PORT_ERR3;
  results_json["C_PORT_ERR4"] = pcap_status.C_PORT_ERR4;
  results_json["C_PORT_ERR5"] = pcap_status.C_PORT_ERR5;
  results_json["C_PORT_ERR_INT"] = pcap_status.C_PORT_ERR_INT;

  serializeJson(results_json, Serial);

 */
  // Serial.print(pcap_results.C0_over_CREF,7);Serial.print(" - ");
  // Serial.print(pcap_results.C1_over_CREF,7);Serial.print(" - ");
  // Serial.print(pcap_results.C2_over_CREF,7);Serial.print(" - ");
  // Serial.print(pcap_results.C3_over_CREF,7);Serial.print(" - ");
  // Serial.print(pcap_results.C4_over_CREF,7);Serial.print(" - ");
  // Serial.print(pcap_results.C5_over_CREF,7);Serial.print(" - ");
  // Serial.print(pcap_results.PT1_over_PTREF,7);Serial.print(" - ");
  // Serial.println(pcap_results.PTInternal_over_PTREF,7);

  // RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
