var SegfaultHandler = require('segfault-handler');
SegfaultHandler.registerHandler("crash.log");

// ONLY INITIALIZATION BEFORE OTHER ELEMENTS
var SerialPort = require("serialport").SerialPort;
var serialPath = "/dev/ttyMFD2";
var serialPort = new SerialPort(serialPath, {
    baudrate: 115200
});

var mraa = require('mraa'); // important note: MRAA is instable and is the cause of 90% of crashes, there be dragons.

moduleDataPath = process.env.MODULE_DATA_DIR || "/media/sdcard/data";
scriptsPath = process.env.SCRIPTS || "/home/root/scripts";
serialNumber = process.env.SERIAL_NUMBER || "mocked-serial-no";
rebootCount = process.env.REBOOT_COUNT || "HARDCODED_VALUE";

rotationalSpeed = (Number(process.env.ROTATION_SPEED)) || 0x14; // up to 127
rotationDuration = (Number(process.env.ROTATION_DURATION)) || 0x14; // up to 127

var express = require('express');
var sdCard = require('fs');


var IMUClass = require('jsupm_lsm9ds0'); // Instantiate an LSM9DS0 using default parameters (bus 1, gyro addr 6b, xm addr 1d)
console.log('IMUClasss', new IMUClass.LSM9DS0())
var AHRS = require('jsupm_ahrs');
var Madgwick = new AHRS.Madgwick();
//console.log('AHRS', )

var horizontalPositionInterruptPin = 11;
var GyroscopeInterruptPin = 12;

var gyrocsopeInterrupt = undefined;
var horizontalPositionInterrupt;
var moduleIsBeingTransportedInterruptPin = 10;
var horizontalPositionInterruptPin = 11;

appMode = process.env.NODE_ENV || "development";

var weAreRotating = 0x60;
var touchDataID = 0;
var motionDataID = 0;
var zAxisThreshold = 0.96;

var powerBoost;
var touchSensor;
var soapSensorIsDamaged = false;
var IMUSensorIsDamaged = false;
gyroAccelCompass = "not initialized"; //GLOBAL VARIABLE
var app;

var soapStatusText = "";
var timeWithUnsavedTouch = 0;
var systemRefreshFrequency = 2000; // in milliseconds
var durationBeforeSleep = 45; // in seconds

var appStateCountdown = 5 * (1000 / systemRefreshFrequency);
var horizontalPositionCheckCountdown = 0.5 * (1000 / systemRefreshFrequency);
var sleepModeCheckCountdown = durationBeforeSleep * (1000 / systemRefreshFrequency);

var xAcceleroValue = new IMUClass.new_floatp();
var yAcceleroValue = new IMUClass.new_floatp();
var zAcceleroValue = new IMUClass.new_floatp();

var xGyroAxis = new IMUClass.new_floatp();
var yGyroAxis = new IMUClass.new_floatp();
var zGyroAxis = new IMUClass.new_floatp();

var xMagnetAxis = new IMUClass.new_floatp();
var yMagnetAxis = new IMUClass.new_floatp();
var zMagnetAxis = new IMUClass.new_floatp();

var currentTime;
var gyroscopeDataText = "";

function gyroInterruptCallBack() {
	// console.log("-ISR GYRO");
}


function logger(msg) {
    serialPort.write(msg + "\n\r", function (err, results) {
    });
    console.log(msg);
}

function getGyroscopeData(currentTime) {
    
    gyroAccelCompass.updateGyroscope();
    gyroAccelCompass.updateAccelerometer();
    gyroAccelCompass.updateMagnetometer();

    gyroAccelCompass.getGyroscope(xGyroAxis, yGyroAxis, zGyroAxis);
    gyroAccelCompass.getAccelerometer(xAcceleroValue, yAcceleroValue, zAcceleroValue);
    gyroAccelCompass.getMagnetometer(xMagnetAxis, yMagnetAxis, zMagnetAxis);
    
    var gx = IMUClass.floatp_value(xGyroAxis);
    var gy = IMUClass.floatp_value(yGyroAxis);  
    var gz = IMUClass.floatp_value(zGyroAxis);
    
    var ax = IMUClass.floatp_value(xAcceleroValue);
    var ay = IMUClass.floatp_value(yAcceleroValue);  
    var az = IMUClass.floatp_value(zAcceleroValue);
    
    var mx = IMUClass.floatp_value(xMagnetAxis);
    var my = IMUClass.floatp_value(yMagnetAxis);  
    var mz = IMUClass.floatp_value(xMagnetAxis);

//    logger("GYRO:")
    
    Madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz)
    
    logger("---------")
    logger('Pitch: ' + Madgwick.getPitch())
    logger('Yaw: ' + Madgwick.getPitch())
    logger('Roll: ' + Madgwick.getRoll())
}


//--------------------------------------------------------------
function setupGyroscope() {

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_CFG_G, 0x08); // enable interrupt only on Y axis (not Latching)
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG1_G, 0x0A); // Y axis enabled only
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG2_G, 0x00);
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG3_G, 0x80);
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG5_G, 0x00);
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_TSH_YH_G, rotationalSpeed); //set threshold for high rotation speed per AXIS, TSH_YH_G is for Y axis only!
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_DURATION_G, (rotationDuration | 0x80)); //set minimum rotation duration to trigger interrupt (based on frequency)


    //showGyrodebugInfo();

    /*gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_INT1_CFG_G,  0x48 ); //0x60 is latched interrupt on Y axis

     gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_CTRL_REG1_G, 0x0F );     //set Frequency of Gyro sensing (ODR) and axis enabled (x,y,z)
     //gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_INT1_DURATION_G, 0x40 ); //set minimum rotation duration to trigger interrupt (based on frequency)
     gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_INT1_TSH_ZH_G, 0x01 );   //set threshold for positive rotation speed

     gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_CTRL_REG2_G, 0x00 ); // normal mode for filtering data
     gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_CTRL_REG3_G, 0x88 ); // interrupt enabled, active high, DRDY enabled
     //gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_GYRO , IMUClass.LSM9DS0.REG_CTRL_REG5_G, 0x00 ); // all default values
     */
}

//--------------------------------------------------------------
function setupAccelerometer() {

    // SETUP GEN 1 FOR Z AXIS HORIZONTAL DETECTION
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_REG, 0x30); //generation on Z high event
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG0_XM, 0x00); //default value
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG1_XM, 0x67); //0x64); //set Frequency of accelero sensing (ODR is 100 Hz) and axis enabled (z)

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG2_XM, 0x21); // Set accelero scale to 2g
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG3_XM, 0x20); //enable pinXM for acclero interrupt
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG5_XM, 0x0); // nothing latch //GEN 1

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_DURATION, 0x33); // set minimum acceleration duration to trigger interrupt
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_THS, 0x32); // set threshold for slightly below 1G value to trigger interrupt (based on 2G scale in accelero)


    //------ Setup interrypt 2 for container transport detection
    gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_XM , IMUClass.LSM9DS0.REG_INT_GEN_2_REG,0x8A); //enable X,Y high acceleration (both needed high for interrupt to happen)
    gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_XM , IMUClass.LSM9DS0.REG_INT_GEN_2_THS,0x64); //100 out of 127 possible on 2G , 100 ~ high 1.5G
    gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_XM , IMUClass.LSM9DS0.REG_INT_GEN_2_DURATION,0x20); //32 out of 127 possible, 32 = 340 ms
}

//--------------------------------------------------------------
function setupMagnetometer() {

    // SETUP GEN 1 FOR Z AXIS HORIZONTAL DETECTION
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_REG, 0x30); //generation on Z high event
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG0_XM, 0x00); //default value
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG1_XM, 0x67); //0x64); //set Frequency of accelero sensing (ODR is 100 Hz) and axis enabled (z)

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG2_XM, 0x21); // Set accelero scale to 2g
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG3_XM, 0x20); //enable pinXM for acclero interrupt
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG5_XM, 0x0); // nothing latch //GEN 1

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_DURATION, 0x33); // set minimum acceleration duration to trigger interrupt
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_THS, 0x32); // set threshold for slightly below 1G value to trigger interrupt (based on 2G scale in accelero)


    //------ Setup interrypt 2 for container transport detection
    gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_XM , IMUClass.LSM9DS0.REG_INT_GEN_2_REG,0x8A); //enable X,Y high acceleration (both needed high for interrupt to happen)
    gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_XM , IMUClass.LSM9DS0.REG_INT_GEN_2_THS,0x64); //100 out of 127 possible on 2G , 100 ~ high 1.5G
    gyroAccelCompass.writeReg( IMUClass.LSM9DS0.DEV_XM , IMUClass.LSM9DS0.REG_INT_GEN_2_DURATION,0x20); //32 out of 127 possible, 32 = 340 ms
}

//--------------------------------------------------------------
function showGyrodebugInfo() {

    /*winston.info("Gyro CFG : 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_CFG_G).toString(16));
    winston.info("Gyro REG1: 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG1_G).toString(16));
    winston.info("Gyro REG2: 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG2_G).toString(16));
    winston.info("Gyro REG3: 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG3_G).toString(16));
    winston.info("Gyro REG4: 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG4_G).toString(16));
    winston.info("Gyro REG5: 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG5_G).toString(16));
    winston.info("Gyro status" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_STATUS_REG_G).toString(16));
    winston.info("Gyro FIFO . 0x" + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_FIFO_CTRL_REG_G).toString(16));
    winston.info("Gyro interrupt source: " + gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_SRC_G).toString(16));
    */
}

//-----------------------------------------------------------------------------------------------------------
function setupMonitoring() {
    
    gyroAccelCompass = new IMUClass.LSM9DS0();

    if (gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_WHO_AM_I_G) != 255) {
        logger("MOTION SENSOR OK");
        gyroAccelCompass.init(); // Initialize the device with default values
        setupGyroscope();
        setupAccelerometer();


        gyrocsopeInterrupt = new mraa.Gpio(GyroscopeInterruptPin);
        gyrocsopeInterrupt.dir(mraa.DIR_IN);

        horizontalPositionInterrupt = new mraa.Gpio(horizontalPositionInterruptPin);
        horizontalPositionInterrupt.dir(mraa.DIR_IN);


        var moduleTransportationInterrupt = new mraa.Gpio(moduleIsBeingTransportedInterruptPin);
        moduleTransportationInterrupt.dir(mraa.DIR_IN);


        // gyrocsopeInterrupt.isr(mraa.EDGE_BOTH, gyroInterruptCallBack);
        // horizontalPositionInterrupt.isr(mraa.EDGE_BOTH, horizontalPositionCallBack);
        // moduleTransportationInterrupt.isr(mraa.EDGE_BOTH, moduleTransportationCallBack);

    } else {
        logger(" !!!!!!!!!!!!!!!!!! NO MOTION SENSOR !!!!!!!!!!!!!!!!");
        IMUSensorIsDamaged = true;
    }

}

// --------------------------------------------------------------------------
//------------------------------- MAIN LOOP ---------------------------------
// --------------------------------------------------------------------------

var justFinishedRecordingMovie = false;

setInterval(function() {

    getGyroscopeData(currentTime);


}, systemRefreshFrequency);

serialPort.on("open", function() {
    serialPort.write("\n\r-----------------------------------------------------------\n\r---------------- Starting monitoring app ----------------\n\r", function(err, results) { //Write data
        setupMonitoring();
    });
});

serialPort.on("error", function() {
    console.log("--SERIAL PORT ENCOUNTERED AN ERROR--");
});

serialPort.on("close", function() {
    console.log("...serial port closed");
});
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------