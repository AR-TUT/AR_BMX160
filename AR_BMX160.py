'''!
  @file AR_BMX160.py
  @brief define BMX160 IMU class infrastructure, the implementation of basic methods
  @Reference Document		  : 	BMX160 - Datasheet
  @Data Sheet Information	: 	Document Version 1.4, Release Date November 2021, Document Number BST-BMX160-DS000-14, Reference Code 0 273 141 190
  @Author 				        :	ArumugaRaja @ AR 
  @Author Mail Id			    :	arpsg17@gmail.com
  @Copyright 			        : 	Any one can reproduce this file
  @File Created Date		  :	25.11.2022
  @url 				            :	https://github.com/AR-TUT/AR_BMX160 
  @Web Address            :   https://arpsg17.weebly.com/
 '''
 
import sys
sys.path.append('../')
import smbus
import time

class BMX160:
    
    BMX160_CHIPID                   = (0x00)    # Page No : 55  
    BMX160_ERROR_REG                = (0x02)    # Page No : 55  
    BMX160_PMU_STATUS               = (0x03)    # Page No : 56  
    BMX160_DATA_MAG_ADDR            = (0x04)    # Page No : 57, MAG_X <7:0> (LSB) 
    BMX160_DATA_GYR_ADDR            = (0x0C)    # Page No : 57, GYR_X <7:0> (LSB) 
    BMX160_DATA_ACC_ADDR            = (0x12)    # Page No : 57, ACC_X <7:0> (LSB) 
    BMX160_SENSORTIME               = (0X18)    # Page No : 58   
    BMX160_STATUS                   = (0x1B)    # Page No : 59      
    BMX160_INT_STATUS_0             = (0x1C)    # Page No : 59 Section 2.11.7  
    BMX160_INT_STATUS_1             = (0x1D)    # Page No : 60 Section 2.11.7  
    BMX160_INT_STATUS_2             = (0x1E)    # Page No : 60 Section 2.11.7  
    BMX160_INT_STATUS_3             = (0x1F)    # Page No : 61 Section 2.11.7  
    BMX160_TEMPERATURE              = (0X20)    # Page No : 61 Section 2.11.8 
    BMX160_FIFO_LENGTH              = (0x22)    # Page No : 62 Section 2.11.9 
    BMX160_FIFO_DATA                = (0x24)    # Page No : 63 Section 2.11.10 
    BMX160_ACC_CONF                 = (0x40)    # Page No : 63 Section 3.11.11 
    BMX160_ACC_RANGE                = (0x41)    # Page No : 64 Section 2.11.12 
    BMX160_GYR_CONF                 = (0x42)    # Page No : 65 Section 2.11.13 
    BMX160_GYR_RANGE                = (0x43)    # Page No : 66 Section 2.11.14 
    BMX160_MAG_CONF                 = (0x44)    # Page No : 66 Section 2.11.15 
    BMX160_FIFO_DOWNS               = (0x45)    # Page N0 : 67 Section 2.11.16 
    BMX160_FIFO_CONFIG_0            = (0x46)    # Page No : 68 Section 2.11.17 
    BMX160_FIFO_CONFIG_1            = (0x47)    # Page No : 68 Section 2.11.17     
    BMX160_MAG_IF_0                 = (0x4C)    # Page No : 69 Section 2.11.18 
    BMX160_MAG_IF_1                 = (0x4D)    # Page No : 69 Section 2.11.18 
    BMX160_MAG_IF_2                 = (0x4E)    # Page No : 69 Section 2.11.18 
    BMX160_MAG_IF_3                 = (0x4F)    # Page No : 69 Section 2.11.18 
    BMX160_INT_EN_0                 = (0x50)    # Page No : 70 Section 2.11.19 
    BMX160_INT_EN_1                 = (0x51)    # Page No : 70 Section 2.11.19 
    BMX160_INT_EN_2                 = (0x52)    # Page No : 70 Section 2.11.19 
    BMX160_INT_OUT_CTRL             = (0x53)    # Page No : 71 Section 2.11.20     
    BMX160_INT_LATCH                = (0x54)    # Page No : 72 Section 2.11.21 
    BMX160_INT_MAP_0                = (0x55)    # Page No : 72 & 73 Section 2.11.22 
    BMX160_INT_MAP_1                = (0x56)    # Page No : 73 Section 2.11.22 
    BMX160_INT_MAP_2                = (0x57)    # Page No : 73 Section 2.11.22 
    BMX160_INT_DATA_0               = (0x58)    # Page No : 74 Section 2.11.23 
    BMX160_INT_DATA_1               = (0x59)    # Page No : 74 Section 2.11.23 
    BMX160_INT_LOWHIGH_0            = (0x5A)    # Page No : 75 Section 2.11.24 
    BMX160_INT_LOWHIGH_1            = (0x5B)    # Page No : 75 Section 2.11.24 
    BMX160_INT_LOWHIGH_2            = (0x5C)    # Page No : 76 Section 2.11.24 
    BMX160_INT_LOWHIGH_3            = (0x5D)    # Page No : 76 Section 2.11.24 
    BMX160_INT_LOWHIGH_4            = (0x5E)    # Page No : 76 Section 2.11.24 
    BMX160_INT_MOTION_0             = (0x5F)    # Page No : 77 Section 2.11.25 
    BMX160_INT_MOTION_1             = (0x60)    # Page No : 78 Section 2.11.25 
    BMX160_INT_MOTION_2             = (0x61)    # Page No : 78 Section 2.11.25 
    BMX160_INT_MOTION_3             = (0x62)    # Page No : 79 Section 2.11.25 
    BMX160_INT_TAP_0                = (0x63)    # Page No : 79 Section 2.11.26 
    BMX160_INT_TAP_1                = (0x64)    # Page No : 80 Section 2.11.26 
    BMX160_INT_ORIENT_0             = (0x65)    # Page No : 80 Section 2.11.27 
    BMX160_INT_ORIENT_1             = (0x66)    # Page No : 81 Section 2.11.27 
    BMX160_INT_FLAT_0               = (0x67)    # Page No : 81 & 82 Section 2.11.28 
    BMX160_INT_FLAT_1               = (0x68)    # Page No : 82 Section 2.11.28 
    BMX160_FOC_CONF                 = (0x69)    # Page No : 82 & 83 Section 2.11.29 
    BMX160_CONF                     = (0x6A)    # Page No : 83 Section 2.11.30 
    BMX160_IF_CONF                  = (0x6B)    # Page No : 83 & 84 Section 2.11.31 
    BMX160_PMU_TRIGGER              = (0x6C)    # Page No : 84 Section 2.11.32 
    BMX160_SELF_TEST                = (0x6D)    # Page No : 85 Section 2.11.33 
    BMX160_NV_CONF                  = (0x70)    # Page No : 86 Section 2.11.34 
    BMX160_OFFSET_0                 = (0x71)    # Page No : 86 Section 2.11.35 
    BMX160_OFFSET_1                 = (0x72)    # Page No : 87 Section 2.11.35 
    BMX160_OFFSET_2                 = (0x73)    # Page No : 87 Section 2.11.35 
    BMX160_OFFSET_3                 = (0x74)    # Page No : 87 Section 2.11.35 
    BMX160_OFFSET_4                 = (0x75)    # Page No : 87 Section 2.11.35 
    BMX160_OFFSET_5                 = (0x76)    # Page No : 87 Section 2.11.35 
    BMX160_OFFSET_6                 = (0x77)    # Page No : 87 Section 2.11.35 
    BMX160_STEP_CNT_0               = (0x78)    # Page No : 87 Section 2.11.36     
    BMX160_STEP_CNT_1               = (0x79)    # Page No : 87 Section 2.11.36 
    BMX160_STEP_CONF_0              = (0x7A)    # Page No : 88 Section 2.11.37 
    BMX160_STEP_CONF_1              = (0x7B)    # Page No : 88 Section 2.11.37  
    
    BMX160_CMD                      = (0x7E)    # Page No : 89 Section 2.11.38  
    BMX160_CMD_START_FOC            = (0x03)    # Page No : 90 Section 2.11.38  
    BMX160_CMD_PROG_NVM             = (0xA0)    # Page No : 90 Section 2.11.38  
    BMX160_CMD_FIFO_FLUSH           = (0xB0)    # Page No : 90 Section 2.11.38 
    BMX160_CMD_INT_RESET            = (0xB1)    # Page No : 90 Section 2.11.38 
    BMX160_CMD_SOFTRESET            = (0xB6)    # Page No : 91 Section 2.11.38 
    BMX160_CMD_STEP_CNT_CLR         = (0xB2)    # Page No : 91 Section 2.11.38 
    
    BMX160_GEOMAG_RESOLUTION        = (0.30)    # Page No : 10 & 11 Table 4 Device Resolution 
    
    BMX160_ACC_MG_LSB_2G    = (0.00006103516)   # Page No : 8 Table 2, ±2g    LSB/g = 16384, Then g/LSB = 1/16384 
    BMX160_ACC_MG_LSB_4G    = (0.00012207031)   # Page No : 8 Table 2, ±4g    LSB/g =  8192, Then g/LSB = 1/8192 
    BMX160_ACC_MG_LSB_8G    = (0.00024414062)   # Page No : 8 Table 2, ±8g    LSB/g =  4096, Then g/LSB = 1/4096 
    BMX160_ACC_MG_LSB_16G   = (0.00048828125)   # Page No : 8 Table 2, ±16g   LSB/g =  2048, Then g/LSB = 1/2048 
    
    BMX160_GYRO_DPS_LSB_2000    = (0.0609756097)    # Page No : 9 Table 3, ±2000dps    LSB/s =  16.4, Then s/LSB = 1/16.4  
    BMX160_GYRO_DPS_LSB_1000    = (0.0304878048)    # Page No : 9 Table 3, ±1000dps    LSB/s =  32.8, Then s/LSB = 1/32.8   
    BMX160_GYRO_DPS_LSB_500     = (0.0152439024)    # Page No : 9 Table 3, ±500dps     LSB/s =  65.6, Then s/LSB = 1/68.6  
    BMX160_GYRO_DPS_LSB_250     = (0.0076219512)    # Page No : 9 Table 3, ±250dps     LSB/s = 131.2, Then s/LSB = 1/137.1 
    BMX160_GYRO_DPS_LSB_125     = (0.0038110975)    # Page No : 9 Table 3, ±125dps     LSB/s = 262.4, Then s/LSB = 1/262.4 
      
    GYR_RANGE_2000DPS       = (0x00)            # Page No : 66, Section 2.11.14 ±2000 is "000" 
    GYR_RANGE_1000DPS       = (0x01)            # Page No : 66, Section 2.11.14 ±1000 is "001" 
    GYR_RANGE_500DPS        = (0x02)            # Page No : 66, Section 2.11.14 ±500  is "010" 
    GYR_RANGE_250DPS        = (0x03)            # Page No : 66, Section 2.11.14 ±250  is "011" 
    GYR_RANGE_125DPS        = (0x04)            # Page No : 66, Section 2.11.14 ±125  is "100" 
        
    ACC_RANGE_2G            = (0x00)
    ACC_RANGE_4G            = (0x01)
    ACC_RANGE_8G            = (0x02)
    ACC_RANGE_16G           = (0x03)
    
    accelRange  =   BMX160_ACC_MG_LSB_2G
    gyroRange   =   BMX160_GYRO_DPS_LSB_250
    
    def __init__(self, bus):
        '''!
          Identify the device I2C address           
        '''
        self.i2cbus = smbus.SMBus(bus)
        self.i2c_addr = 0x68
        time.sleep(0.16)
    
    def begin(self):
        '''!
          Initialization of i2c.
          Return the I2C Connection status of IMU           
        '''
        if not self.scan():
            return False
        else:
            self.soft_reset()
            self.write_bmx_reg(self.BMX160_CMD, 0x11)   # Page No : 90, Section 2.11.38, Table 29, Accelerometer to normal mode
            time.sleep(0.05)
            self.write_bmx_reg(self.BMX160_CMD, 0x15)   # Page No : 90, Section 2.11.38, Table 29, Gyroscope to normal mode 
            time.sleep(0.1)
            self.write_bmx_reg(self.BMX160_CMD, 0x19)   # Page No : 90, Section 2.11.38, Table 29, Mangetometer to normal mode 
            time.sleep(0.01)
            self.set_magn_conf()
            return True

    def set_low_power(self):
        '''!
          Disabled the the magn, gyro sensor to reduce power consumption
        '''
        self.soft_reset()
        time.sleep(0.1)
        self.set_magn_conf()
        time.sleep(0.1)
        self.write_bmx_reg(self.BMX160_CMD, 0x12)   # Page No : 90, Section 2.11.38, Table 29, Accelerometer to low power mode
        time.sleep(0.1)
        self.write_bmx_reg(self.BMX160_CMD, 0x17)   # Page No : 90, Section 2.11.38, Table 29, Gyroscope to suspend mode 
        time.sleep(0.1)
        self.write_bmx_reg(self.BMX160_CMD, 0x1B)   # Page No : 90, Section 2.11.38, Table 29, Mangetometer to low power mode 
        time.sleep(0.1)

    def wake_up(self):
        '''!
          Enabled the the magn, gyro sensor
        '''
        self.soft_reset()
        time.sleep(0.1)
        self.set_magn_conf()
        time.sleep(0.1)
        self.write_bmx_reg(self.BMX160_CMD, 0x11) # Page No : 90, Section 2.11.38, Table 29, Accelerometer to normal mode
        time.sleep(0.1)
        self.write_bmx_reg(self.BMX160_CMD, 0x15) # Page No : 90, Section 2.11.38, Table 29, Gyroscope to normal mode 
        time.sleep(0.1)
        self.write_bmx_reg(self.BMX160_CMD, 0x19) # Page No : 90, Section 2.11.38, Table 29, Mangetometer to normal mode 
        time.sleep(0.1)

    def soft_reset(self):
        '''!
          Reset bmx160 hardware
          Return reset status           
        '''
        data = self.BMX160_CMD_SOFTRESET
        self.write_bmx_reg(self.BMX160_CMD, data)
        time.sleep(0.015)         
        return True

    def set_magn_conf(self):
        '''!
          Set magnetometer Configuration
        '''
        self.write_bmx_reg(self.BMX160_MAG_IF_0, 0x80)
        time.sleep(0.05)
        self.write_bmx_reg(self.BMX160_MAG_IF_3, 0x01)
        self.write_bmx_reg(self.BMX160_MAG_IF_2, 0x4B)
        self.write_bmx_reg(self.BMX160_MAG_IF_3, 0x04)
        self.write_bmx_reg(self.BMX160_MAG_IF_2, 0x51)
        self.write_bmx_reg(self.BMX160_MAG_IF_3, 0x0E)
        self.write_bmx_reg(self.BMX160_MAG_IF_2, 0x52)
        
        self.write_bmx_reg(self.BMX160_MAG_IF_3, 0x02)
        self.write_bmx_reg(self.BMX160_MAG_IF_2, 0x4C)
        self.write_bmx_reg(self.BMX160_MAG_IF_1, 0x42)
        self.write_bmx_reg(self.BMX160_MAG_CONF, 0x08)
        self.write_bmx_reg(self.BMX160_MAG_IF_0, 0x03)
        time.sleep(0.05)

    def set_gyro_range(self, bits):
        '''!
          Set gyroscope angular rate range and resolution.
          GyroRange_125DPS      Gyroscope sensitivity at 125dps
          GyroRange_250DPS      Gyroscope sensitivity at 250dps
          GyroRange_500DPS      Gyroscope sensitivity at 500dps
          GyroRange_1000DPS     Gyroscope sensitivity at 1000dps
          GyroRange_2000DPS     Gyroscope sensitivity at 2000dps
        '''
        if bits == 0:
            self.gyroRange = self.BMX160_GYRO_DPS_LSB_125
        elif bits == 1:
            self.gyroRange = self.BMX160_GYRO_DPS_LSB_250
        elif bits == 2:
            self.gyroRange = self.BMX160_GYRO_DPS_LSB_500
        elif bits == 3:
            self.gyroRange = self.BMX160_GYRO_DPS_LSB_1000
        elif bits == 4:
            self.gyroRange = self.BMX160_GYRO_DPS_LSB_2000
        else:
            self.gyroRange = self.BMX160_GYRO_DPS_LSB_250

    def set_accel_range(self, bits):
        '''!
          Selection of the accelerometer g-range.
          Page No : 8 Table 2, ±2g    LSB/g = 16384, Then g/LSB = 1/16384  = 0.00006103516 mg
          Page No : 8 Table 2, ±4g    LSB/g =  8192, Then g/LSB = 1/8192   = 0.00012207031 mg 
          Page No : 8 Table 2, ±8g    LSB/g =  4096, Then g/LSB = 1/4096   = 0.00024414062 mg
          Page No : 8 Table 2, ±16g   LSB/g =  2048, Then g/LSB = 1/2048   = 0.00048828125 mg        
        '''
        if bits == 0:
            self.accelRange = self.BMX160_ACC_MG_LSB_2G
        elif bits == 1:
            self.accelRange = self.BMX160_ACC_MG_LSB_4G
        elif bits == 2:
            self.accelRange = self.BMX160_ACC_MG_LSB_8G
        elif bits == 3:
            self.accelRange = self.BMX160_ACC_MG_LSB_16G
        else:
            self.accelRange = self.BMX160_ACC_MG_LSB_2G

    def get_all_data(self):
        '''!
          Get the Magnatometer, Gyroscope, and Accelerometer data 
          Return all data
        '''
        data = self.read_bmx_reg(self.BMX160_DATA_MAG_ADDR)
        if (data[1] & 0x80):
            magnx = - 0x10000 + ((data[1] << 8) | (data[0]))
        else:
            magnx =  (data[1] << 8) | (data[0])
        if (data[3] & 0x80):
            magny = - 0x10000 + ((data[3] << 8) | (data[2]))
        else:
            magny =  (data[3] << 8) | (data[2])
        if (data[5] & 0x80):
            magnz = - 0x10000 + ((data[5] << 8) | (data[4]))
        else:
            magnz =  (data[5] << 8) | (data[4])

        if (data[9] & 0x80):
            gyrox = - 0x10000 + ((data[9] << 8) | (data[8]))
        else:
            gyrox =  (data[9] << 8) | (data[8])
        if (data[11] & 0x80):
            gyroy = - 0x10000 + ((data[11] << 8) | (data[10]))
        else:
            gyroy =  (data[11] << 8) | (data[10])
        if (data[13] & 0x80):
            gyroz = - 0x10000 + ((data[13] << 8) | (data[12]))
        else:
            gyroz =  (data[13] << 8) | (data[12])

        if (data[15] & 0x80):
            accelx = - 0x10000 + ((data[15] << 8) | (data[14]))
        else:
            accelx =  (data[15] << 8) | (data[14])
        if (data[17] & 0x80):
            accely = - 0x10000 + ((data[17] << 8) | (data[16]))
        else:
            accely =  (data[17] << 8) | (data[16])
        if (data[19] & 0x80):
            accelz = - 0x10000 + ((data[19] << 8) | (data[18]))
        else:
            accelz =  (data[19] << 8) | (data[18])
        
        magnx *= self.BMX160_GEOMAG_RESOLUTION
        magny *= self.BMX160_GEOMAG_RESOLUTION
        magnz *= self.BMX160_GEOMAG_RESOLUTION
        
        gyrox *= self.gyroRange
        gyroy *= self.gyroRange
        gyroz *= self.gyroRange
        
        accelx *= self.accelRange * 9.8
        accely *= self.accelRange * 9.8
        accelz *= self.accelRange * 9.8
        out_put = []
        out_put.append(magnx)
        out_put.append(magny)
        out_put.append(magnz)
        out_put.append(gyrox)
        out_put.append(gyroy)
        out_put.append(gyroz)
        out_put.append(accelx)
        out_put.append(accely)
        out_put.append(accelz)
        return out_put

    def write_bmx_reg(self, register, value):
        '''!
          Write data to the BMX160 register           
        '''
        self.i2cbus.write_byte_data(self.i2c_addr, register, value)

    def read_bmx_reg(self, register):
        '''!
          Read BMX register data
          Return data
        '''
        return self.i2cbus.read_i2c_block_data(self.i2c_addr, register)

    def scan(self):
        '''!
          I2C scan function
          Return scan result
          Retval True sensor exist
          Retval False There is no sensor
        '''
        try:
            self.i2cbus.read_byte(self.i2c_addr)
            return True
        except:
            print("I2C init fail")
            return False
