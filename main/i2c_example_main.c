/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"

/**
 * TEST CODE BRIEF for clock_card10
 *
 */

#define DATA_LENGTH                        512              /*!<Data buffer length for test buffer*/
#define RW_TEST_LENGTH                     129              /*!<Data length for r/w test, any value from 0-DATA_LENGTH*/
#define DELAY_TIME_BETWEEN_ITEMS_MS        1234             /*!< delay time between different test items */

#define I2C_EXAMPLE_SLAVE_SCL_IO           26               /*!<gpio number for i2c slave clock  */
#define I2C_EXAMPLE_SLAVE_SDA_IO           25               /*!<gpio number for i2c slave data */
#define I2C_EXAMPLE_SLAVE_NUM              I2C_NUM_0        /*!<I2C port number for slave dev */
#define I2C_EXAMPLE_SLAVE_TX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave tx buffer size */
#define I2C_EXAMPLE_SLAVE_RX_BUF_LEN       (2*DATA_LENGTH)  /*!<I2C slave rx buffer size */
//I2C gpio number in the latest CLK10
#define I2C_EXAMPLE_MASTER_SCL_IO          5//19               /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO          10//18               /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_1        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000           /*!< I2C master clock frequency */

#define RDS_RADIO                          0x11
#define SI5319                             0x68
#define BH1750_SENSOR_ADDR                 0x20             /*!< slave address for BH1750 sensor */
#define BH1750_CMD_START                   0x23             /*!< Command to set measure mode */
#define ESP_SLAVE_ADDR                     SI5319 //0x28             /*!< ESP32 slave address, you can set any 7bit value */
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define NUM_REGS                           31*2

SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave register
 *
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_register(i2c_port_t i2c_num, uint8_t reg, uint8_t* buf, size_t bufsize)
{

    esp_err_t ret;
    if (bufsize == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);

    ret =i2c_master_write_byte (cmd, reg, ACK_CHECK_EN);

                if (ret != ESP_OK) {
                            printf("%s: error sending i2c register ...skip...\n", esp_err_to_name(ret));
                            return ret;
                        }

    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, buf, NACK_VAL);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}



static esp_err_t i2c_master_write_register(i2c_port_t i2c_num, uint8_t reg, uint8_t value){

    esp_err_t ret=0;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ret=i2c_master_write_byte(cmd, ( ESP_SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
            if (ret != ESP_OK) {
                        printf("%s: No ack, SI5319 not connected...skip...\n", esp_err_to_name(ret));
                        return ret;
                    }

        printf ("i2c command sent successfully");

        ret =i2c_master_write_byte (cmd, reg, ACK_CHECK_EN);

                if (ret != ESP_OK) {
                            printf("%s: error sending i2c register ...skip...\n", esp_err_to_name(ret));
                            return ret;
                        }


        ret = i2c_master_write_byte (cmd, value, ACK_CHECK_EN);

                if (ret != ESP_OK) {
                            printf("%s: error sending register value ...skip... \n", esp_err_to_name(ret));
                            return ret;
                        }


        i2c_master_stop(cmd);

            ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);

                   if (ret != ESP_OK) {
                                    printf("%s: trigger sending queued commands failed ...skip... \n", esp_err_to_name(ret));
                                    return ret;
                                }


        i2c_cmd_link_delete(cmd);

        return ret;



}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_registers(i2c_port_t i2c_num, uint8_t* data_wr, size_t size)
{

    esp_err_t ret=0;
    i2c_cmd_handle_t cmd;

    for (int i=0; i<size; i+=2){


        ret = i2c_master_write_register (i2c_num, data_wr[i], data_wr[i+1]);
        if (ret != ESP_OK) {
                            printf("%s: error sending i2c register ...skip...\n", esp_err_to_name(ret));
                            return ret;
                        }

    }


    return ret;
}



/**
 * @brief i2c master initialization
 */
static void i2c_example_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief i2c slave initialization
 */
static void i2c_example_slave_init()
{
    int i2c_slave_port = I2C_EXAMPLE_SLAVE_NUM;
    i2c_config_t conf_slave;
    conf_slave.sda_io_num = I2C_EXAMPLE_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_EXAMPLE_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    i2c_driver_install(i2c_slave_port, conf_slave.mode,
                       I2C_EXAMPLE_SLAVE_RX_BUF_LEN,
                       I2C_EXAMPLE_SLAVE_TX_BUF_LEN, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t* buf, int len)
{
    int i;
    for (i = 0; i < len; i++) {
        printf("%02x ", buf[i]);
        if (( i + 1 ) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
}


void app_main()
{


    i2c_example_master_init();


          printf("*******************\n");
          printf("SI5319 configuration\n");


          //registers description
          //https://www.silabs.com/documents/public/data-sheets/Si5319.pdf
          //software for register configuration
          //https://www.silabs.com/products/development-tools/timing/clock/si5319-evb-development-kit



          printf("25MHZ->28.8MHZ\n");


	 uint8_t data_wr[] = {

                                             0x00, 0x14,
                                             0x02, 0x92,
                                             0x03, 0x15,
                                             0x05, 0x6d,
                                             0x06, 0x2a,
                                             0x08, 0x00,
                                             0x0a, 0x00,
                                             0x0b, 0x40,
                                             0x13, 0x2c,
                                             0x14, 0x3e,
                                             0x16, 0xdf,
                                             0x17, 0x1f,
                                             0x18, 0x3f,
                                             0x19, 0x20,
                                             0x1f, 0x00,
                                             0x20, 0x00,
                                             0x21, 0x25,
                                             0x28, 0x80,
                                             0x29, 0x02,
                                             0x2a, 0xab,
                                             0x2b, 0x00,
                                             0x2c, 0x00,
                                             0x2d, 0x18,
                                             0x2e, 0x00,
                                             0x2f, 0x00,
                                             0x30, 0x18,
                                             0x83, 0x1f,
                                             0x84, 0x02,
                                             0x8a, 0x0f,
                                             0x8b, 0xff,
                                             0x88, 0x40


	};


/*
          printf("28.8MHZ->28.8MHZ\n");

                   uint8_t data_wr[] = {

                                             0x00, 0x14,
                                             0x02, 0xa2,
                                             0x03, 0x15,
                                             0x05, 0x6d,
                                             0x06, 0x2a,
                                             0x08, 0x00,
                                             0x0a, 0x00,
                                             0x0b, 0x40,
                                             0x13, 0x2c,
                                             0x14, 0x3e,
                                             0x16, 0xdf,
                                             0x17, 0x1f,
                                             0x18, 0x3f,
                                             0x19, 0x80,
                                             0x1f, 0x00,
                                             0x20, 0x00,
                                             0x21, 0x17,
                                             0x28, 0x20,
                                             0x29, 0x02,
                                             0x2a, 0x3f,
                                             0x2b, 0x00,
                                             0x2c, 0x00,
                                             0x2d, 0x0e,
                                             0x2e, 0x00,
                                             0x2f, 0x00,
                                             0x30, 0x0e,
                                             0x83, 0x1f,
                                             0x84, 0x02,
                                             0x8a, 0x0f,
                                             0x8b, 0xff,
                                             0x88, 0x40

                                             };

*/

          /*

          printf("28.8MHZ->40MHZ\n");


          uint8_t data_wr[] = {

                                   0x00, 0x14,
                                   0x02, 0xa2,
                                   0x03, 0x15,
                                   0x05, 0x6d,
                                   0x06, 0x2a,
                                   0x08, 0x00,
                                   0x0a, 0x00,
                                   0x0b, 0x40,
                                   0x13, 0x2c,
                                   0x14, 0x3e,
                                   0x16, 0xdf,
                                   0x17, 0x1f,
                                   0x18, 0x3f,
                                   0x19, 0xe0,
                                   0x1f, 0x00,
                                   0x20, 0x00,
                                   0x21, 0x0b,
                                   0x28, 0xe0,
                                   0x29, 0x00,
                                   0x2a, 0xf9,
                                   0x2b, 0x00,
                                   0x2c, 0x00,
                                   0x2d, 0x0e,
                                   0x2e, 0x00,
                                   0x2f, 0x00,
                                   0x30, 0x0e,
                                   0x83, 0x1f,
                                   0x84, 0x02,
                                   0x8a, 0x0f,
                                   0x8b, 0xff,
                                   0x88, 0x40

                                   };


*/

/*

               printf("10MHZ->40MHZ\n");
               uint8_t data_wr[] = {
                                        0x00, 0x14,
                                        0x02, 0xa2,
                                        0x03, 0x15,
                                        0x05, 0x6d,
                                        0x06, 0x2a,
                                        0x08, 0x00,
                                        0x0a, 0x00,
                                        0x0b, 0x40,
                                        0x13, 0x2c,
                                        0x14, 0x3e,
                                        0x16, 0xdf,
                                        0x17, 0x1f,
                                        0x18, 0x3f,
                                        0x19, 0x60,
                                        0x1f, 0x00,
                                        0x20, 0x00,
                                        0x21, 0x11,
                                        0x28, 0x40,
                                        0x29, 0x01,
                                        0x2a, 0xa3,
                                        0x2b, 0x00,
                                        0x2c, 0x00,
                                        0x2d, 0x04,
                                        0x2e, 0x00,
                                        0x2f, 0x00,
                                        0x30, 0x04,
                                        0x83, 0x1f,
                                        0x84, 0x02,
                                        0x8a, 0x0f,
                                        0x8b, 0xff,
                                        0x88, 0x40

                                        };

  */


            size_t length = sizeof(data_wr)/sizeof(data_wr[0]);


            int ret = i2c_master_write_registers( I2C_EXAMPLE_MASTER_NUM, data_wr, length);
            if (ret == ESP_OK) {

                printf ("clock card initilized successfully \n");

            }
            else {printf ("error is occured %s", esp_err_to_name(ret));

            }

            /*
            example of reading SI5319 register
            */

            uint8_t reg = 0x30; //register to read
            uint8_t buf[1] = {0x00}; //buffer
            size_t sizebuf = sizeof(buf)/sizeof(buf[0]); //size


            i2c_master_read_register (I2C_EXAMPLE_MASTER_NUM, reg, buf, sizebuf);

            disp_buf (buf, 1);


}

