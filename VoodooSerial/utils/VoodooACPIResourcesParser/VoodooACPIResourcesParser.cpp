//
//  VoodooACPIResourcesParser.cpp
//  VoodooI2C
//
//  Created by CoolStar on 8/15/17.
//  Copyright © 2017 CoolStar. All rights reserved.
//

// Demo Standalone Program: https://ghostbin.com/paste/tqt73

#include <IOKit/IOLib.h>
#include <string.h>

#include "VoodooACPIResourcesParser.hpp"
#include "linuxirq.hpp"

VoodooACPIResourcesParser::VoodooACPIResourcesParser() {
    found_gpio_interrupts = false;
    found_i2c = false;
}

void VoodooACPIResourcesParser::parseACPISerialBus(UInt8 const* res, UInt32 offset, UInt32 sz) {
    if (found_i2c || found_uart)
        return;
    if (offset >= sz)
        return;
    UInt8 opcode = res[offset];
    if (opcode != 0x8e)
        return;
    
    UInt16 len;
    memcpy(&len, res + offset + 1, sizeof(UInt16));
    
    UInt8 bustype = res[offset + 5];
    UInt8 flags = res[offset + 6];
    
    UInt16 tflags;
    memcpy(&tflags, res + offset + 7, sizeof(UInt16));
    
    UInt16 datalen;
    memcpy(&datalen, res + offset + 10, sizeof(UInt16));
    
    if (bustype == BUS_TYPE_I2C) {
        found_i2c = true;
        
        i2c_info.resource_consumer = (flags >> 1) & 0x1;
        i2c_info.device_initiated = flags & 0x1;
        i2c_info.address_mode_10Bit = tflags & 0x1;
        
        UInt32 busspeed;
        memcpy(&busspeed, res + offset + 12, sizeof(UInt32));
        i2c_info.bus_speed = busspeed;
        
        UInt16 address;
        memcpy(&address, res + offset + 16, sizeof(UInt16));
        i2c_info.address = address;
    } else if (bustype == BUS_TYPE_UART) {
        found_uart = true;
        
        uart_info.resource_consumer = flags>>1 & 0x1;
        uart_info.device_initiated = flags & 0x1;
        
        uart_info.flow_control = tflags & 0x03;
        uart_info.stop_bits = tflags>>2 & 0x03;
        uart_info.data_bits = tflags>>4 & 0x07;
        uart_info.big_endian = tflags>>7 & 0x01;
        
        UInt32 baudrate;
        memcpy(&baudrate, res + offset + 12, sizeof(UInt32));
        uart_info.baudrate = baudrate;
        
        UInt16 rx_fifo;
        memcpy(&rx_fifo, res + offset + 16, sizeof(UInt16));
        uart_info.rx_fifo = rx_fifo;
        
        UInt16 tx_fifo;
        memcpy(&tx_fifo, res + offset + 18, sizeof(UInt16));
        uart_info.tx_fifo = tx_fifo;
        
        UInt8 parity;
        memcpy(&parity, res + offset + 20, sizeof(UInt8));
        uart_info.parity = parity;
        
        UInt8 lines_enabled;
        memcpy(&lines_enabled, res + offset + 21, sizeof(UInt8));
        uart_info.dtd_enabled = lines_enabled>>2 & 0x01;
        uart_info.ri_enabled  = lines_enabled>>3 & 0x01;
        uart_info.dsr_enabled = lines_enabled>>4 & 0x01;
        uart_info.dtr_enabled = lines_enabled>>5 & 0x01;
        uart_info.cts_enabled = lines_enabled>>6 & 0x01;
        uart_info.rts_enabled = lines_enabled>>7 & 0x01;
    }
}

void VoodooACPIResourcesParser::parseACPIGPIO(UInt8 const* res, UInt32 offset, UInt32 sz) {
    if (found_gpio_interrupts)
        return;
    if (offset >= sz)
        return;
    
    UInt8 opcode = res[offset];
    if (opcode != 0x8c)
        return;
    
    UInt16 len;
    memcpy(&len, res + offset + 1, sizeof(UInt16));
    
    UInt8 gpio_type = res[offset + 4];
    UInt8 flags = res[offset + 5];
    
    UInt8 gpio_flags = res[offset + 7];
    
    UInt8 pin_config = res[offset + 9];
    
    UInt16 pin_offset;
    memcpy(&pin_offset, res + offset + 14, sizeof(UInt16));
    
    UInt16 resourceOffset;
    memcpy(&resourceOffset, res + offset + 17, sizeof(UInt16));
    
    UInt16 vendorOffset;
    memcpy(&vendorOffset, res + offset + 19, sizeof(UInt16));
    
    UInt16 pin_number;
    memcpy(&pin_number, res + offset + pin_offset, sizeof(UInt16));
    
    if (pin_number == 0xFFFF) // pinNumber 0xFFFF is invalid
        return;
    
    if (gpio_type == 0) {
        // GPIOInt
        found_gpio_interrupts = true;
        
        gpio_interrupts.resource_consumer = flags & 0x1;
        gpio_interrupts.level_interrupt = !(gpio_flags & 0x1);
        
        gpio_interrupts.interrupt_polarity = (gpio_flags >> 1) & 0x3;
        
        gpio_interrupts.shared_interrupt = (gpio_flags >> 3) & 0x1;
        gpio_interrupts.wake_interrupt = (gpio_flags >> 4) & 0x1;
        
        gpio_interrupts.pin_config = pin_config;
        gpio_interrupts.pin_number = pin_number;
        
        int irq = 0;
        if (gpio_interrupts.level_interrupt) {
            switch (gpio_interrupts.interrupt_polarity) {
                case 0:
                    irq = IRQ_TYPE_LEVEL_HIGH;
                    break;
                case 1:
                    irq = IRQ_TYPE_LEVEL_LOW;
                    break;
                default:
                    irq = IRQ_TYPE_LEVEL_LOW;
                    break;
            }
        } else {
            switch (gpio_interrupts.interrupt_polarity) {
                case 0:
                    irq = IRQ_TYPE_EDGE_FALLING;
                    break;
                case 1:
                    irq = IRQ_TYPE_EDGE_RISING;
                    break;
                case 2:
                    irq = IRQ_TYPE_EDGE_BOTH;
                    break;
                default:
                    irq = IRQ_TYPE_EDGE_FALLING;
                    break;
            }
        }
        gpio_interrupts.irq_type = irq;
    } else if (gpio_type == 1) {
        // GPIOIo
        found_gpio_io = true;
        
        gpio_io.resource_consumer = flags & 0x1;
        gpio_io.io_restriction = gpio_flags & 0x3;
        gpio_io.sharing = (gpio_flags >> 3) & 0x1;
        
        gpio_io.pin_config = pin_config;
        gpio_io.pin_number = pin_number;
    }
}

UInt32 VoodooACPIResourcesParser::parseACPIResources(UInt8 const* res, UInt32 offset, UInt32 sz) {
    if (offset >= sz)
        return offset;
    
    UInt8 opcode = res[offset];
    
    UInt16 len;
    memcpy(&len, res + offset + 1, sizeof(UInt16));
    
    if (opcode == 0x8c) {
        if (found_gpio_interrupts)
            return offset;
        parseACPIGPIO(res, offset, sz);
    } else if (opcode == 0x8e) {
        if (found_i2c || found_uart)
            return offset;
        parseACPISerialBus(res, offset, sz);
    }
    
    offset += (len + 3);
    return parseACPIResources(res, offset, sz);
}
