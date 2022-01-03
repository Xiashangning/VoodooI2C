## VoodooI2C

VoodooI2C is a project consisting of macOS kernel extensions that add support for I2C bus devices.
This kext is intended to be installed by anyone whose computer requires some form of I2C support. It consists of I2C controller drivers and is responsible for publishing device nubs to the IOService plane.

## Current Status

The following Intel I2C controllers are fully supported:

1. `INT33C2` and `INT33C3` - Haswell era
2. `INT3432` and `INT3433` - Broadwell era
3. `pci8086,9d60`, `pci8086,9d61`, `pci8086,9d62` and `pci8086,9d63` - Skylake era
4. `pci8086,a160`, `pci8086,a161`, `pci8086,a162` and `pci8086,a163` - Kaby Lake era
5. `pci8086,9de8`, `pci8086,9de9`, `pci8086,9dea` and `pci8086,9deb` - Cannon Lake/Whiskey Lake era
6. `pci8086,a368`, `pci8086,a369`, `pci8086,a36a` and `pci8086,a36b` - Coffee Lake era
7. `pci8086,2e8`, `pci8086,2e9`, `pci8086,2ea`, `pci8086,2eb`, `pci8086,6e8`, `pci8086,6e9`, `pci8086,6ea` and `pci8086,6eb`- Comet Lake era
8. `pci8086,34e8`, `pci8086,34e9`, `pci8086,34ea` and `pci8086,34eb` - Ice Lake era

The following device classes are fully supported:

1. I2C-HID devices
2. ELAN devices
3. FTE devices

Note that there is sometimes an overlap between device classes. For example, some ELAN devices may also be I2C-HID devices.

## VoodooUART

The following UART devices are supported:

34a88086  Surface Pro 7
9d278086  Surface Book 2

Interrupt mode is not realised due to the failure of interrupt registration

## License

This program is protected by the GPL license. Please refer to the `LICENSE.txt` file for more information

