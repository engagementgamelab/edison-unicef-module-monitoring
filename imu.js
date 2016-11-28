var _APP_TIMEOUT = 50000;

        //300000
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
var targz = require('tar.gz');

var IMUClass = require('jsupm_lsm9ds0'); // Instantiate an LSM9DS0 using default parameters (bus 1, gyro addr 6b, xm addr 1d)
var AHRS = require('jsupm_ahrs');
var Madgwick = new AHRS.Madgwick();

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
var systemRefreshFrequency = 100; // in milliseconds
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

var zeroVals = [0, 0, 0, 0, 0, 0];

var zMagnetAxis = new IMUClass.new_floatp();

var currentTime;
var gyroscopeDataText = "";

var pushButtonLightPin = 13;
var pushButtonLight = new mraa.Gpio(pushButtonLightPin);

var initWebService = function () {
    app = express();
    app.set('port', (process.env.MONITORING_PORT || 3001));
    
    app.get('/', function (req, res) {
        
        res.header('Access-Control-Allow-Origin', '*');
        res.header('Access-Control-Allow-Methods', 'GET');
        res.header('Access-Control-Allow-Headers', 'Content-Type');

        var sensorsOverallStatus = "OK";
        var errorStatus = "";

        if (gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_WHO_AM_I_G) === 255) {
            errorStatus += "| Gyroscope unreachable "; // if chip failed return false all the time
            sensorsOverallStatus += " | Gyroscope FAIL";
        }
        if (gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_WHO_AM_I_XM) === 255) {
            errorStatus += " | Accelerometer unreachable. "; // if chip failed return false all the time
            sensorsOverallStatus += " | Accelerometer FAIL.";
        }
        
        res.send('<script>timeLeft=' + _APP_TIMEOUT + '; setInterval(function() {' +
                'timeLeft-=1000;' + 
                'if(timeLeft < 0) { document.body.innerHTML = "App is closed! Reboot device to launch app again." } ' + 
                ' else { document.getElementById("timer").innerHTML = timeLeft/1000 + " seconds left"; } }, 1000);</script>' +
                '<a href="/download">Download CSV Archive</a> <br /><b>IMU Status:</b><br />' + 
                sensorsOverallStatus + errorStatus + 
                '<p>Time left before this app will shutdown: <span id="timer"></span></p>');
        
    });

    app.get('/download', function (req, res) {

        var today = new Date();
        var day = (today.getMonth() + 1) + "-" + today.getDate() + "-" + today.getFullYear().toString();
        var fileName = 'imu_data_'+ day +'.tar.gz';
        var filePath = __dirname + '/' + fileName;
        
        res.header('Access-Control-Allow-Origin', '*');
        res.header('Access-Control-Allow-Methods', 'GET');
        res.header('Access-Control-Allow-Headers', 'Content-Type');

        targz().compress('/media/sdcard/sensor_data', filePath, function(err) {

              if(err)
                console.error('Something is wrong ', err.stack);
            
            res.type('application/tar+gzip');
            res.setHeader('Content-disposition', 'attachment; filename=' + fileName);
            res.sendFile(filePath);

        });

    });
    app.listen(app.get('port'), function () {
        console.info('Monitoring listening on port ' + app.get('port'));

        // Turn on LED
        pushButtonLight.write(1);

        // Shut app down after timeout
        setInterval(function() { 
            pushButtonLight.write(0);

            process.exit(); }, _APP_TIMEOUT);
    });
}

//--------------------------------------------------------------
function setupGyroscope() {

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_CFG_G,  0x08); // enable interrupt only on Y axis (not Latching)
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG1_G, 0x0A);// Y axis enabled only
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG2_G, 0x00);
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG3_G, 0x80);
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_CTRL_REG5_G, 0x00);
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_TSH_YH_G, rotationalSpeed);//set threshold for high rotation speed per AXIS, TSH_YH_G is for Y axis only!
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_INT1_DURATION_G, (rotationDuration | 0x80)); //set minimum rotation duration to trigger interrupt (based on frequency)
    
}

//--------------------------------------------------------------
function setupAccelerometer() {

    // SETUP GEN 1 FOR Z AXIS HORIZONTAL DETECTION
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_REG, 0x20); //generation on Z high event
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG0_XM, 0x00); //default value
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG1_XM, 0x67); //0x64); //set Frequency of accelero sensing (ODR is 100 Hz) and axis enabled (z)

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG2_XM, 0x00); // Set accelero scale to 2g
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG3_XM, 0x20); //enable pinXM for acclero interrupt
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_CTRL_REG5_XM, 0x0); // nothing latch //GEN 1

    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_DURATION, 0x2F); // set minimum acceleration duration to trigger interrupt
    gyroAccelCompass.writeReg(IMUClass.LSM9DS0.DEV_XM, IMUClass.LSM9DS0.REG_INT_GEN_1_THS, 0x3E); // set threshold for slightly below 1G value to trigger interrupt (based on 2G scale in accelero)

}

//-----------------------------------------------------------------------------------------------------------
function setupMonitoring() {


    pushButtonLight.dir(mraa.DIR_OUT);
    
    gyroAccelCompass = new IMUClass.LSM9DS0();

    if (gyroAccelCompass.readReg(IMUClass.LSM9DS0.DEV_GYRO, IMUClass.LSM9DS0.REG_WHO_AM_I_G) != 255) {
        
        gyroAccelCompass.init(); // Initialize the device with default values
        setupGyroscope();
        setupAccelerometer();

    } else {
        IMUSensorIsDamaged = true;
    }

}

setupMonitoring();

serialPort.on("error", function() {
    console.log("--SERIAL PORT ENCOUNTERED AN ERROR--");
});

serialPort.on("close", function() {
    console.log("...serial port closed");
});

initWebService();
// --------------------------------------------------------------------------
// --------------------------------------------------------------------------