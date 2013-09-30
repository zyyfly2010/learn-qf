
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;

    /* Toplevel class names for drivers: */
    class UARTDriver;
    class I2CDriver;

    class SPIDeviceDriver;
    class SPIDeviceManager;

    class AnalogSource;
    class AnalogIn;
    class Storage;
    class ConsoleDriver;
    class DigitalSource;
    class GPIO;
    class RCInput;
    class RCOutput;
    class Scheduler;
    class Semaphore;
    
    class Util;

    /* Utility Classes */
    class Print;
    class Stream;
    class BetterStream;

    /* Typdefs for function pointers (Procedure, Timed Procedure) */
    typedef void(*Proc)(void);
    typedef void(*TimedProc)(void *);

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

    enum SPIDevice {
        SPIDevice_Dataflash,
        SPIDevice_ADS7844,
        SPIDevice_MS5611,
        SPIDevice_MPU6000,
        SPIDevice_ADNS3080_SPI0,
        SPIDevice_ADNS3080_SPI3
    };

}

#if __AVR__ && __GNUC__ == 4 && __GNUC_MINOR__ > 6
#define AP_HAL_TIMEDPROC(func) reinterpret_cast<AP_HAL::TimedProc>(func)
#else
#define AP_HAL_TIMEDPROC(func) (AP_HAL::TimedProc)(func)
#endif

#endif // __AP_HAL_NAMESPACE_H__
