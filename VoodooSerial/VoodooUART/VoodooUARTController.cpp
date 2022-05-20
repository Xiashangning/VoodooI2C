//
//  VoodooUARTController.cpp
//  BigSurface
//
//  Created by Xavier on 2021/9/23.
//  Copyright © 2021 Xia Shangning. All rights reserved.
//

#include "VoodooUARTController.hpp"

#define LOG(str, ...) IOLog("%s::%s " str "\n", getName(), physical_device.name, ##__VA_ARGS__)

#define CONFIGURED(a) a?"YES":"NO"

#define super IOService
OSDefineMetaClassAndStructors(VoodooUARTController, IOService);

bool VoodooUARTController::init(OSDictionary* properties) {
    if (!super::init(properties))
        return false;
    memset(&bus, 0, sizeof(VoodooUARTBus));
    memset(&physical_device, 0, sizeof(VoodooUARTPhysicalDevice));
    physical_device.state = UART_IDLE;

    return true;
}

void VoodooUARTController::free() {
    super::free();
}

IOReturn VoodooUARTController::mapMemory() {
    if (physical_device.device->getDeviceMemoryCount() == 0) {
        return kIOReturnDeviceError;
    } else {
        physical_device.mmap = physical_device.device->mapDeviceMemoryWithIndex(0);
        if (!physical_device.mmap) return kIOReturnDeviceError;
        return kIOReturnSuccess;
    }
}

IOReturn VoodooUARTController::unmapMemory() {
    OSSafeReleaseNULL(physical_device.mmap);
    return kIOReturnSuccess;
}

void VoodooUARTController::resetDevice() {
    writeRegister(0, LPSS_PRIV + LPSS_PRIV_RESETS);
    writeRegister(LPSS_PRIV_RESETS_FUNC | LPSS_PRIV_RESETS_IDMA, LPSS_PRIV + LPSS_PRIV_RESETS);
    
    //Clear FIFO disable all interrupts, they will be renabled in prepareCommunication.
    fcr = 0;
    mcr = 0;
    ier = 0;
    writeRegister(UART_FCR_ENABLE_FIFO, DW_UART_FCR);
    writeRegister(UART_FCR_ENABLE_FIFO | UART_FCR_CLEAR_RX_FIFO | UART_FCR_CLEAR_TX_FIFO, DW_UART_FCR);
    writeRegister(0, DW_UART_FCR);
    readRegister(DW_UART_RX);
    writeRegister(0, DW_UART_IER);
    
    //Clear the interrupt registers.
    readRegister(DW_UART_LSR);
    readRegister(DW_UART_RX);
    readRegister(DW_UART_IIR);
    readRegister(DW_UART_MSR);
    readRegister(DW_UART_USR);
    
    //DMA TBD Here
    
    toggleInterruptType(UART_IER_ENABLE_MODEM_STA_INT, true);
}

IOReturn VoodooUARTController::configureDevice() {
    LOG("Set PCI power state D0");
    IOPCIDevice *pci_device = physical_device.device;
    pci_device->enablePCIPowerManagement(kPCIPMCSPowerStateD0);

    /* Apply Forcing D0 patch from VoodooI2C*/
    UInt16 oldPowerStateWord = pci_device->configRead16(0x80 + 0x4);
    UInt16 newPowerStateWord = oldPowerStateWord & ~0x3;
    pci_device->configWrite16(0x80 + 0x4, newPowerStateWord);

    pci_device->setBusMasterEnable(true);
    pci_device->setMemoryEnable(true);
    
    if (mapMemory() != kIOReturnSuccess) {
        LOG("Could not map memory");
        return kIOReturnError;
    }
    startUARTClock();
    return kIOReturnSuccess;
}

IOService* VoodooUARTController::probe(IOService* provider, SInt32* score) {
    if (!super::probe(provider, score))
        return nullptr;
    
    physical_device.device = OSDynamicCast(IOPCIDevice, provider);
    if (!physical_device.device)
        return nullptr;
    
    physical_device.name = physical_device.device->getName();
    physical_device.device_id = physical_device.device->configRead16(0x02);
    LOG("device id: %x", physical_device.device_id);

    return this;
}

void VoodooUARTController::startUARTClock() {
    if (physical_device.device_id == 0x34a8)
        writeRegister(UART_UPDATE_CLK | CALC_CLK(8, 15) | UART_ENABLE_CLK, LPSS_PRIV + LPSS_UART_CLK);
    else if (physical_device.device_id == 0x9d27)
        writeRegister(UART_UPDATE_CLK | CALC_CLK(2, 5) | UART_ENABLE_CLK, LPSS_PRIV + LPSS_UART_CLK);
    else
        LOG("Unknown UART type");
}

void VoodooUARTController::stopUARTClock() {
    writeRegister(0, LPSS_PRIV + LPSS_UART_CLK);
}

bool VoodooUARTController::start(IOService* provider) {
    UInt32 reg;
    if (!super::start(provider))
        return false;
    
    lock = IOLockAlloc();
    if (!lock) {
        LOG("Could not allocate UART bus lock");
        goto exit;
    }
    
    work_loop = IOWorkLoop::workLoop();
    if (!work_loop) {
        LOG("Could not get work loop");
        goto exit;
    }
    command_gate = IOCommandGate::commandGate(this);
    if (!command_gate || (work_loop->addEventSource(command_gate) != kIOReturnSuccess)) {
        LOG("Could not open command gate");
        goto exit;
    }
    transmit_action = OSMemberFunctionCast(IOCommandGate::Action, this, &VoodooUARTController::transmitDataGated);
    
    if (!physical_device.device->open(this)) {
        LOG("Could not open provider");
        goto exit;
    }
    if (configureDevice() != kIOReturnSuccess) {
        LOG("Configure Device Failed!");
        goto exit;
    }
    resetDevice();
    
    fifo_size = DW_UART_CPR_FIFO_SIZE(readRegister(DW_UART_CPR));
    bus.tx_buffer = new VoodooUARTMessage;
    bus.tx_buffer->length = 0;
    bus.rx_buffer = new UInt8[fifo_size];
    reg = readRegister(DW_UART_UCV);
    LOG("Designware UART version %c.%c%c", (reg >> 24) & 0xff, (reg >> 16) & 0xff, (reg >> 8) & 0xff);
//    reg = readRegister(DW_UART_CPR);
//    int abp_width = reg&UART_CPR_ABP_DATA_WIDTH ? (reg&UART_CPR_ABP_DATA_WIDTH)*16 : 8;
//    LOG("UART capabilities:");
//    LOG("    ABP_DATA_WIDTH      : %d", abp_width);
//    LOG("    AFCE_MODE           : %s", CONFIGURED(reg&UART_CPR_AFCE_MODE));
//    LOG("    THRE_MODE           : %s", CONFIGURED(reg&UART_CPR_THRE_MODE));
//    LOG("    SIR_MODE            : %s", CONFIGURED(reg&UART_CPR_SIR_MODE));
//    LOG("    SIR_LP_MODE         : %s", CONFIGURED(reg&UART_CPR_SIR_LP_MODE));
//    LOG("    ADDITIONAL_FEATURES : %s", CONFIGURED(reg&UART_CPR_ADDITIONAL_FEATURES));
//    LOG("    FIFO_ACCESS         : %s", CONFIGURED(reg&UART_CPR_FIFO_ACCESS));
//    LOG("    FIFO_STAT           : %s", CONFIGURED(reg&UART_CPR_FIFO_STAT));
//    LOG("    SHADOW              : %s", CONFIGURED(reg&UART_CPR_SHADOW));
//    LOG("    ENCODED_PARMS       : %s", CONFIGURED(reg&UART_CPR_ENCODED_PARMS));
//    LOG("    DMA_EXTRA           : %s", CONFIGURED(reg&UART_CPR_DMA_EXTRA));
//    LOG("    FIFO_SIZE           : %d", fifo_size);
//    if (physical_device.device->registerInterrupt(0, this, OSMemberFunctionCast(IOInterruptAction, this, &VoodooUARTController::handleInterrupt), 0) != kIOReturnSuccess) {
//        LOG("Too bad that we can only use timer");
    is_polling = true;
    interrupt_simulator = IOTimerEventSource::timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &VoodooUARTController::simulateInterrupt));
    if (!interrupt_simulator) {
        LOG("No! Even timer cannot be created!");
        goto exit;
    }
    work_loop->addEventSource(interrupt_simulator);
//    }
    startInterrupt();
    
    PMinit();
    physical_device.device->joinPMtree(this);
    registerPowerDriver(this, MyIOPMPowerStates, kIOPMNumberPowerStates);
    
    physical_device.device->retain();
    registerService();
    return true;
exit:
    releaseResources();
    return false;
}

void VoodooUARTController::stop(IOService *provider) {
    stopCommunication();
    PMstop();
    releaseResources();
    super::stop(provider);
}

void VoodooUARTController::toggleInterruptType(UInt32 type, bool enable) {
    if (enable)
        ier |= type;
    else
        ier &= ~type;
    writeRegister(ier, DW_UART_IER);
}

inline UInt32 VoodooUARTController::readRegister(int offset) {
    if (physical_device.mmap != 0) {
         IOVirtualAddress address = physical_device.mmap->getVirtualAddress();
         if (address != 0)
             return *(const volatile UInt32 *)(address + offset);
     }
     return 0;
}

inline void VoodooUARTController::writeRegister(UInt32 value, int offset) {
    if (physical_device.mmap != 0) {
        IOVirtualAddress address = physical_device.mmap->getVirtualAddress();
        if (address != 0)
            *(volatile UInt32 *)(address + offset) = value;
    }
}

void VoodooUARTController::releaseResources() {
    stopInterrupt();
    if (!is_polling) {
        physical_device.device->unregisterInterrupt(0);
    } else {
        if (interrupt_simulator) {
            interrupt_simulator->cancelTimeout();
            interrupt_simulator->disable();
            work_loop->removeEventSource(interrupt_simulator);
            OSSafeReleaseNULL(interrupt_simulator);
        }
    }
    if (interrupt_source) {
        work_loop->removeEventSource(interrupt_source);
        OSSafeReleaseNULL(interrupt_source);
    }
    if (bus.tx_buffer)
        delete bus.tx_buffer;
    if (bus.rx_buffer)
        delete[] bus.rx_buffer;
    unmapMemory();
    if (physical_device.device->isOpen(this))
        physical_device.device->close(this);
    if (command_gate) {
        work_loop->removeEventSource(command_gate);
        OSSafeReleaseNULL(command_gate);
    }
    OSSafeReleaseNULL(work_loop);
    if (lock)
        IOLockFree(lock);
    OSSafeReleaseNULL(physical_device.device);
}

IOReturn VoodooUARTController::setPowerState(unsigned long whichState, IOService* whatDevice) {
    if (whatDevice != this)
        return kIOPMAckImplied;

    // ensure that we are not in the middle of a data transmission
    IOLockLock(lock);
    if (whichState == kIOPMPowerOff) {  // index of kIOPMPowerOff state in VoodooUARTIOPMPowerStates
        if (physical_device.state != UART_SLEEP) {
            physical_device.state = UART_SLEEP;
            stopCommunication();
            stopInterrupt();
            stopUARTClock();
            unmapMemory();
            LOG("Going to sleep");
        }
    } else {
        if (physical_device.state == UART_SLEEP) {
            if (configureDevice() != kIOReturnSuccess)
                LOG("Wakeup Config Failed");
            resetDevice();
            physical_device.state = UART_IDLE;
            prepareCommunication();
            startInterrupt();
            LOG("Woke up");
        }
    }
    IOLockUnlock(lock);
    return kIOPMAckImplied;
}

IOReturn VoodooUARTController::requestConnect(OSObject *owner, MessageHandler _handler, UInt32 baud_rate, UInt8 data_bits, UInt8 stop_bits, UInt8 parity) {
    if (target)
        return kIOReturnNoResources;
    if (!owner || !_handler)
        return kIOReturnError;

    target = owner;
    handler = _handler;
    
    bus.baud_rate = baud_rate;
    bus.data_bits = data_bits;
    bus.stop_bits = stop_bits;
    bus.parity = parity;
    
    return prepareCommunication();
}

void VoodooUARTController::requestDisconnect(OSObject *owner) {
    if (target == owner) {
        target = nullptr;
        handler = nullptr;
        
        stopCommunication();
    }
}

IOReturn VoodooUARTController::transmitData(UInt8 *buffer, UInt16 length) {
    if (!ready)
        return kIOReturnNotReady;
    
    IOReturn ret = kIOReturnSuccess;
    IOLockLock(lock);
    for (int tries=0; tries < 5; tries++) {
        ret = command_gate->runAction(transmit_action, buffer, &length);
        if (ret == kIOReturnBusy) {
            LOG("Busy");
            IOSleep(random()&0x03 + 8);
        } else
            break;
    }
    IOLockUnlock(lock);
    return ret;
}

IOReturn VoodooUARTController::transmitDataGated(UInt8* buffer, UInt16* length) {
    AbsoluteTime abstime, deadline;
    IOReturn sleep;
    
    if (bus.tx_buffer->length)
        return kIOReturnBusy;
    
    nanoseconds_to_absolutetime(50000000, &abstime); // 50ms
    clock_absolutetime_interval_to_deadline(abstime, &deadline);
    
    bus.tx_buffer->buffer = buffer;
    bus.tx_buffer->length = *length;
    toggleInterruptType(UART_IER_ENABLE_TX_EMPTY_INT | UART_IER_ENABLE_THRE_INT_MODE, true);
    interrupt_simulator->setTimeoutUS(1);
    sleep = command_gate->commandSleep(&write_complete, deadline, THREAD_INTERRUPTIBLE);
    
    if (sleep == THREAD_TIMED_OUT) {
        LOG("Timeout waiting for transfer request");
        return kIOReturnTimeout;
    }
    return kIOReturnSuccess;
}

IOReturn VoodooUARTController::prepareCommunication() {
    if (ready)
        return kIOReturnStillOpen;
    
    UInt16 divisor = 1;
    
    UInt32 lcr = 0;
    if (bus.data_bits >= UART_DATABITS_9) {
        LOG("unsupported data bits 9!");
        return kIOReturnUnsupported;
    }
    if (bus.stop_bits == UART_STOPBITS_1_5 && bus.data_bits != UART_DATABITS_5) {
        LOG("unsupported stop bits 1.5 when data bits is not 5!");
        return kIOReturnUnsupported;
    }
    if (bus.stop_bits == UART_STOPBITS_NONE) {
        LOG("unsupported stop bits 0!");
        return kIOReturnUnsupported;
    }
    lcr |= bus.data_bits;
    if (bus.stop_bits == UART_STOPBITS_1_5 || bus.stop_bits == UART_STOPBITS_2)
        lcr |= UART_LCR_STOP;
    if (bus.parity != UART_PARITY_NONE) {
        lcr |= UART_LCR_PARITY;
        if (bus.parity == UART_PARITY_EVEN)
            lcr |= UART_LCR_EPAR;
        else if (bus.parity != UART_PARITY_ODD) {
            LOG("unsupported parity type %d!", bus.parity);
            return kIOReturnUnsupported;
        }
    }
    
    if (waitUntilNotBusy() != kIOReturnSuccess)
        return kIOReturnBusy;
    writeRegister(UART_LCR_DLAB, DW_UART_LCR);
    writeRegister(divisor>>8, DW_UART_DLH);
    writeRegister(divisor & 0xFF, DW_UART_DLL);
    writeRegister(lcr, DW_UART_LCR);
    
    // enable FIFO and set threshold trigger
    fcr = UART_FCR_ENABLE_FIFO | UART_FCR_T_TRIG_HALF | UART_FCR_R_TRIG_HALF;
    writeRegister(fcr, DW_UART_FCR);
    
    // Request to send
    mcr |= UART_MCR_RTS | UART_MCR_AFC;
    writeRegister(mcr, DW_UART_MCR);
    for (int tries=0; tries < 10; tries++) {
        if (readRegister(DW_UART_MSR)&UART_MSR_CTS) {
            LOG("Received Clear To Send Signal!");
            break;
        }
        IODelay(10);
    }
    // Enable rx interrupt
    toggleInterruptType(UART_IER_ENABLE_RX_AVAIL_INT | UART_IER_ENABLE_ERR_INT, true);
    ready = true;
    
    return kIOReturnSuccess;
}

void VoodooUARTController::stopCommunication() {
    if (!ready)
        return;
    waitUntilNotBusy();
    
    writeRegister(UART_LCR_DLAB, DW_UART_LCR);
    writeRegister(0, DW_UART_DLH);
    writeRegister(0, DW_UART_DLL);
    writeRegister(0, DW_UART_LCR);
    
    writeRegister(fcr | UART_FCR_CLEAR_RX_FIFO | UART_FCR_CLEAR_TX_FIFO, DW_UART_FCR);
    fcr = 0;
    writeRegister(fcr, DW_UART_FCR);
    
    //Clear the interrupt registers.
    readRegister(DW_UART_LSR);
    readRegister(DW_UART_RX);
    readRegister(DW_UART_IIR);
    readRegister(DW_UART_MSR);
    readRegister(DW_UART_USR);
    ier = 0;
    writeRegister(ier, DW_UART_IER);
    
    mcr = 0;
    writeRegister(mcr, DW_UART_MCR);
    
    ready = false;
}

IOReturn VoodooUARTController::waitUntilNotBusy() {
    int timeout = UART_TIMEOUT * 150, firstDelay = 10;

    while (readRegister(DW_UART_USR) & UART_USR_BUSY) {
        if (timeout <= 0) {
            LOG("Warning: Timeout waiting for UART not to be busy");
            return kIOReturnBusy;
        }
        timeout--;
        if (firstDelay-- >= 0)
            IODelay(100);
        else
            IOSleep(1);
    }
    return kIOReturnSuccess;
}

// Unused due to failure in registering hardware interrupt.
void VoodooUARTController::handleInterrupt(OSObject* target, void* refCon, IOService* provider, int source) {
    
}

void VoodooUARTController::simulateInterrupt(OSObject* owner, IOTimerEventSource* timer) {
    if (physical_device.state == UART_SLEEP)
        return;
    if (!ready) {
        interrupt_simulator->setTimeoutMS(UART_LONG_IDLE_TIMEOUT);
        return;
    }
    
    AbsoluteTime    cur_time;
    clock_get_uptime(&cur_time);
    UInt32 status = readRegister(DW_UART_IIR)&UART_IIR_INT_MASK;
    UInt32 lsr = readRegister(DW_UART_LSR);
    if (bus.tx_buffer->length) {
        physical_device.state = UART_ACTIVE;
        last_activate_time = cur_time;
        UInt32 data;
        if (!(lsr&UART_LSR_TX_FIFO_FULL)) {
            do {
                data = *bus.tx_buffer->buffer++;
                writeRegister(data, DW_UART_TX);
                lsr = readRegister(DW_UART_LSR);
            } while (!(lsr&UART_LSR_TX_FIFO_FULL) && --bus.tx_buffer->length);
            if (!bus.tx_buffer->length) {
                toggleInterruptType(UART_IER_ENABLE_TX_EMPTY_INT, false);
                command_gate->commandWakeup(&write_complete);
            }
        }
    }
    if (lsr & (UART_LSR_DATA_READY|UART_LSR_BREAK_INT)) {
        physical_device.state = UART_ACTIVE;
        last_activate_time = cur_time;
        UInt8 *pos = bus.rx_buffer;
        UInt16 length = 0;
        do {
            *pos++ = readRegister(DW_UART_RX);
            length++;
        } while (readRegister(DW_UART_LSR) & (UART_LSR_DATA_READY|UART_LSR_BREAK_INT) && length < fifo_size);
        if (handler) {
            handler(target, this, bus.rx_buffer, length);
        }
    } else if (status==UART_IIR_RX_LINE_STA_INT) {
        LOG("Receiving data error! LSR: 0x%x", readRegister(DW_UART_LSR));
    } else if (status==UART_IIR_MODEM_STA_INT) {
        LOG("Modem status changed! MSR: 0x%x", readRegister(DW_UART_MSR));
    } else if (status==UART_IIR_BUSY_DETECT) {
        LOG("Detected writing LCR while busy! USR: 0x%x", readRegister(DW_UART_USR));
    }
    
    UInt64 nsecs;
    SUB_ABSOLUTETIME(&cur_time, &last_activate_time);
    absolutetime_to_nanoseconds(cur_time, &nsecs);
    if (physical_device.state == UART_ACTIVE) {
        if (nsecs < 100000000) // < 0.1s
            interrupt_simulator->setTimeoutMS(UART_ACTIVE_TIMEOUT);
        else
            physical_device.state=UART_IDLE;
    }
    if (physical_device.state == UART_IDLE) {
        if (nsecs < 1000000000) // < 1s
            interrupt_simulator->setTimeoutMS(UART_IDLE_TIMEOUT);
        else
            physical_device.state=UART_LONG_IDLE;
    }
    if (physical_device.state == UART_LONG_IDLE)
        interrupt_simulator->setTimeoutMS(UART_LONG_IDLE_TIMEOUT);
}

IOReturn VoodooUARTController::startInterrupt() {
    if (is_interrupt_enabled)
        return kIOReturnStillOpen;
    
    if (is_polling) {
        interrupt_simulator->enable();
        interrupt_simulator->setTimeoutMS(UART_IDLE);
    } else
        physical_device.device->enableInterrupt(0);
    is_interrupt_enabled = true;
    return kIOReturnSuccess;
}

void VoodooUARTController::stopInterrupt() {
    if (is_interrupt_enabled) {
        if (is_polling) {
            interrupt_simulator->cancelTimeout();
            interrupt_simulator->disable();
        } else
            physical_device.device->disableInterrupt(0);
        is_interrupt_enabled = false;
    }
}
