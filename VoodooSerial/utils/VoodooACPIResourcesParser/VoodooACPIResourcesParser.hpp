//
//  VoodooACPIResourcesParser.hpp
//  VoodooI2C
//
//  Created by CoolStar on 8/15/17.
//  Copyright © 2017 CoolStar. All rights reserved.
//

#include <stdint.h>

#ifndef VoodooACPIResourcesParser_hpp
#define VoodooACPIResourcesParser_hpp

#define BUS_TYPE_I2C                1
#define BUS_TYPE_SPI                2
#define BUS_TYPE_UART               3

#define UART_FLOWCRTL_NONE          0x00
#define UART_FLOWCRTL_HARDWARE      0x01
#define UART_FLOWCRTL_SOFTWARE      0x02

#define UART_STOPBITS_NONE          0x00
#define UART_STOPBITS_1             0x01
#define UART_STOPBITS_1_5           0x02
#define UART_STOPBITS_2             0x03

#define UART_DATABITS_5             0x00
#define UART_DATABITS_6             0x01
#define UART_DATABITS_7             0x02
#define UART_DATABITS_8             0x03
#define UART_DATABITS_9             0x04

#define UART_PARITY_NONE            0x00
#define UART_PARITY_EVEN            0x01
#define UART_PARITY_ODD             0x02
#define UART_PARITY_MARK            0x03
#define UART_PARITY_SPACE           0x04


struct i2c_info {
    bool resource_consumer;
    bool device_initiated;
    bool address_mode_10Bit;
    
    UInt32 bus_speed;
    UInt16 address;
};

struct uart_info {
    bool resource_consumer;
    bool device_initiated;
    
    UInt8 flow_control;
    UInt8 stop_bits;
    UInt8 data_bits;
    bool big_endian;
    UInt32 baudrate;
    UInt16 rx_fifo;
    UInt16 tx_fifo;
    UInt8 parity;
    
    bool dtd_enabled;
    bool ri_enabled;
    bool dsr_enabled;
    bool dtr_enabled;
    bool cts_enabled;
    bool rts_enabled;
};

struct gpio_int_info {
    bool resource_consumer;
    bool level_interrupt;
    
    UInt8 interrupt_polarity;
    
    bool shared_interrupt;
    bool wake_interrupt;
    
    UInt8 pin_config;
    UInt16 pin_number;
    
    int irq_type;
};

struct gpio_io_info {
    bool resource_consumer;
    UInt8 io_restriction;
    bool sharing;
    
    UInt8 pin_config;
    UInt16 pin_number;
};

class VoodooACPIResourcesParser {
public:
    bool found_i2c;
    bool found_uart;
    bool found_gpio_interrupts;
    bool found_gpio_io;
    
    i2c_info i2c_info;
    uart_info uart_info;
    gpio_int_info gpio_interrupts;
    gpio_io_info gpio_io;
    
    VoodooACPIResourcesParser();
    UInt32 parseACPIResources(UInt8 const* res, UInt32 offset, UInt32 sz);
private:
    void parseACPISerialBus(UInt8 const* res, UInt32 offset, UInt32 sz);
    void parseACPIGPIO(UInt8 const* res, UInt32 offset, UInt32 sz);
};

#endif /* VoodooACPIResourcesParser_hpp */
