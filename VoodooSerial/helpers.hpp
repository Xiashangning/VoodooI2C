//
//  helpers.hpp
//  VoodooI2C
//
//  Created by Alexandre on 31/07/2017.
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#ifndef helpers_hpp
#define helpers_hpp

#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>

#ifndef EXPORT
#define EXPORT __attribute__((visibility("default")))
#endif

#define kIOPMPowerOff                       0
#define kIOPMNumberPowerStates              2

#define BIT(nr) (1UL << (nr))

#define AbsoluteTime_to_scalar(x)    (*(uint64_t *)(x))

#define SUB_ABSOLUTETIME(t1, t2)                \
    (AbsoluteTime_to_scalar(t1) -=                \
        AbsoluteTime_to_scalar(t2))

#define LPSS_PRIV                   0x200
#define LPSS_PRIV_RESETS            0x04
#define LPSS_PRIV_RESETS_FUNC       0x3
#define LPSS_PRIV_RESETS_IDMA       BIT(2)
#define LPSS_PRIV_REMAP_ADDR        0x40
#define LPSS_PRIVATE_CLOCK_GATING   0x800

inline const char* getMatchedName(IOService* provider) {
    OSData* data = OSDynamicCast(OSData, provider->getProperty("name"));
    return data ? (const char *)(data->getBytesNoCopy()) : "(null)";
}

inline void setOSDictionaryNumber(OSDictionary* dictionary, const char * key, UInt32 number) {
    if (OSNumber* os_number = OSNumber::withNumber(number, 32)) {
        dictionary->setObject(key, os_number);
        os_number->release();
    }
}

enum VoodooI2CState {
    kVoodooI2CStateOff = 0,
    kVoodooI2CStateOn = 1
};

static IOPMPowerState myIOPMPowerStates[kIOPMNumberPowerStates] = {
    {1, kIOPMPowerOff, kIOPMPowerOff, kIOPMPowerOff, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMPowerOn, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
};

#define LOG(str, ...)   IOLog("%s::" str "\n", getName(), ##__VA_ARGS__)

#ifdef DEBUG
#define DBG_LOG(str, ...)   LOG("dbg-" str, ##__VA_ARGS__)
#else
#define DBG_LOG(str, ...)   ;
#endif

#endif /* helpers_hpp */
