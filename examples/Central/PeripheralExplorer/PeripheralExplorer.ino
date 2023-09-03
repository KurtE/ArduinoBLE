/*
  Peripheral Explorer

  This example scans for Bluetooth® Low Energy peripherals until one with a particular name ("LED")
  is found. Then connects, and discovers + prints all the peripheral's attributes.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use it with another board that is compatible with this library and the
  Peripherals -> LED example.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // begin initialization
  //BLE.debug(Serial);
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }

  Serial.println("Bluetooth® Low Energy Central - Peripheral Explorer");

  // start scanning for peripherals
  BLE.scan();
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // see if peripheral is a LED
    static const char device_name[] = "Xbox Wireless Controller";
    //static const char device_name[] = "Surface Arc Mouse";
    if (peripheral.localName() == device_name) {
      // stop scanning
      BLE.stopScan();

      explorerPeripheral(peripheral);

      // peripheral disconnected, we are done
      while (1) {
        // do nothing
      }
    }
  }
}

void explorerPeripheral(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // read and print device name of peripheral
  Serial.println();
  Serial.print("Device name: ");
  Serial.println(peripheral.deviceName());
  Serial.print("Appearance: 0x");
  Serial.println(peripheral.appearance(), HEX);
  Serial.println();

  // loop the services of the peripheral and explore each
  for (int i = 0; i < peripheral.serviceCount(); i++) {
    BLEService service = peripheral.service(i);

    exploreService(service);
  }

  Serial.println();

  // we are done exploring, disconnect
  Serial.println("Disconnecting ...");
  peripheral.disconnect();
  Serial.println("Disconnected");
}

uint32_t HexStroTInt(const char *sz) {
  uint32_t val = 0;
  while (*sz) {
    uint8_t c = *sz++;
    if ((c >= '0') && (c <= '9')) val = val * 16 + c - '0';
    else if ((c >= 'a') && (c <= 'f')) val = val * 16 + 10 + (c - 'a');
    else if ((c >= 'A') && (c <= 'F')) val = val * 16 + 10 + (c - 'A');
    else break;
  }
  return val;
}


const char *MapUUIDToName(const char *uuid_str) {
// Only allow this on boards that have enough memory/storage  
#if !defined(ARDUINO_AVR_UNO_WIFI_REV2)
  if (strlen(uuid_str) < 6) {
    uint32_t uuid_int = HexStroTInt(uuid_str);
    switch (uuid_int) {
      // Services
      case 0x1800: return "Generic Access service";
      case 0x1801: return "Generic Attribute service";
      case 0x1802: return "Immediate Alert service";
      case 0x1803: return "Link Loss service";
      case 0x1804: return "Tx Power service";
      case 0x1805: return "Current Time service";
      case 0x1806: return "Reference Time Update service";
      case 0x1807: return "Next DST Change service";
      case 0x1808: return "Glucose service";
      case 0x1809: return "Health Thermometer service";
      case 0x180A: return "Device Information service";
      case 0x180D: return "Heart Rate service";
      case 0x180E: return "Phone Alert Status service";
      case 0x180F: return "Battery service";
      case 0x1810: return "Blood Pressure service";
      case 0x1811: return "Alert Notification service";
      case 0x1812: return "Human Interface Device service";
      case 0x1813: return "Scan Parameters service";
      case 0x1814: return "Running Speed and Cadence service";
      case 0x1815: return "Automation IO service";
      case 0x1816: return "Cycling Speed and Cadence service";
      case 0x1818: return "Cycling Power service";
      case 0x1819: return "Location and Navigation service";
      case 0x181A: return "Environmental Sensing service";
      case 0x181B: return "Body Composition service";
      case 0x181C: return "User Data service";
      case 0x181D: return "Weight Scale service";
      case 0x181E: return "Bond Management service";
      case 0x181F: return "Continuous Glucose Monitoring service";
      case 0x1820: return "Internet Protocol Support service";
      case 0x1821: return "Indoor Positioning service";
      case 0x1822: return "Pulse Oximeter service";
      case 0x1823: return "HTTP Proxy service";
      case 0x1824: return "Transport Discovery service";
      case 0x1825: return "Object Transfer service";
      case 0x1826: return "Fitness Machine service";
      case 0x1827: return "Mesh Provisioning service";
      case 0x1828: return "Mesh Proxy service";
      case 0x1829: return "Reconnection Configuration service";
      case 0x183A: return "Insulin Delivery service";
      case 0x183B: return "Binary Sensor service";
      case 0x183C: return "Emergency Configuration service";
      case 0x183D: return "Authorization Control service";
      case 0x183E: return "Physical Activity Monitor service";
      case 0x1843: return "Audio Input Control service";
      case 0x1844: return "Volume Control service";
      case 0x1845: return "Volume Offset Control service";
      case 0x1846: return "Coordinated Set Identification service";
      case 0x1847: return "Device Time service";
      case 0x1848: return "Media Control service";
      case 0x1849: return "Generic Media Control service";
      case 0x184A: return "Constant Tone Extension service";
      case 0x184B: return "Telephone Bearer service";
      case 0x184C: return "Generic Telephone Bearer service";
      case 0x184D: return "Microphone Control service";
      case 0x184E: return "Audio Stream Control service";
      case 0x184F: return "Broadcast Audio Scan service";
      case 0x1850: return "Published Audio Capabilities service";
      case 0x1851: return "Basic Audio Announcement service";
      case 0x1852: return "Broadcast Audio Announcement service";
      case 0x1853: return "Common Audio service";
      case 0x1854: return "Hearing Aid service";
      case 0x1855: return "TMAS service";

      // Declarations 
      case 0x2800: return "Primary Service";
      case 0x2801: return "Secondary Service";
      case 0x2802: return "Include";
      case 0x2803: return "Characteristic";

      // Descriptions:
      case 0x2900: return "Characteristic Extended Properties";
      case 0x2901: return "Characteristic User Description";
      case 0x2902: return "Client Characteristic Configuration";
      case 0x2903: return "Server Characteristic Configuration";
      case 0x2904: return "Characteristic Presentation Format";
      case 0x2905: return "Characteristic Aggregate Format";
      case 0x2906: return "Valid Range";
      case 0x2907: return "External Report Reference";
      case 0x2908: return "Report Reference";
      case 0x2909: return "Number of Digitals";
      case 0x290A: return "Value Trigger Setting";
      case 0x290B: return "Environmental Sensing Configuration";
      case 0x290C: return "Environmental Sensing Measurement";
      case 0x290D: return "Environmental Sensing Trigger Setting";
      case 0x290E: return "Time Trigger Setting";
      case 0x290F: return "Complete BREDR Transport Block Data";
      
      //Characteristics
      case 0x2A00: return "Device Name";
      case 0x2A01: return "Appearance";
      case 0x2A02: return "Peripheral Privacy Flag";
      case 0x2A03: return "Reconnection Address";
      case 0x2A04: return "Peripheral Preferred Connection Parameters";
      case 0x2A05: return "Service Changed";
      case 0x2A06: return "Alert Level";
      case 0x2A07: return "Tx Power Level";
      case 0x2A08: return "Date Time";
      case 0x2A09: return "Day of Week";
      case 0x2A0A: return "Day Date Time";
      case 0x2A0C: return "Exact Time 256";
      case 0x2A0D: return "DST Offset";
      case 0x2A0E: return "Time Zone";
      case 0x2A0F: return "Local Time Information";
      case 0x2A11: return "Time with DST";
      case 0x2A12: return "Time Accuracy";
      case 0x2A13: return "Time Source";
      case 0x2A14: return "Reference Time Information";
      case 0x2A16: return "Time Update Control Point";
      case 0x2A17: return "Time Update State";
      case 0x2A18: return "Glucose Measurement";
      case 0x2A19: return "Battery Level";
      case 0x2A1C: return "Temperature Measurement";
      case 0x2A1D: return "Temperature Type";
      case 0x2A1E: return "Intermediate Temperature";
      case 0x2A21: return "Measurement Interval";
      case 0x2A22: return "Boot Keyboard Input Report";
      case 0x2A23: return "System ID";
      case 0x2A24: return "Model Number String";
      case 0x2A25: return "Serial Number String";
      case 0x2A26: return "Firmware Revision String";
      case 0x2A27: return "Hardware Revision String";
      case 0x2A28: return "Software Revision String";
      case 0x2A29: return "Manufacturer Name String";
      case 0x2A2A: return "IEEE 1107320601 Regulatory Certification Data List";
      case 0x2A2B: return "Current Time";
      case 0x2A2C: return "Magnetic Declination";
      case 0x2A31: return "Scan Refresh";
      case 0x2A32: return "Boot Keyboard Output Report";
      case 0x2A33: return "Boot Mouse Input Report";
      case 0x2A34: return "Glucose Measurement Context";
      case 0x2A35: return "Blood Pressure Measurement";
      case 0x2A36: return "Intermediate Cuff Pressure";
      case 0x2A37: return "Heart Rate Measurement";
      case 0x2A38: return "Body Sensor Location";
      case 0x2A39: return "Heart Rate Control Point";
      case 0x2A3F: return "Alert Status";
      case 0x2A40: return "Ringer Control Point";
      case 0x2A41: return "Ringer Setting";
      case 0x2A42: return "Alert Category ID Bit Mask";
      case 0x2A43: return "Alert Category ID";
      case 0x2A44: return "Alert Notification Control Point";
      case 0x2A45: return "Unread Alert Status";
      case 0x2A46: return "New Alert";
      case 0x2A47: return "Supported New Alert Category";
      case 0x2A48: return "Supported Unread Alert Category";
      case 0x2A49: return "Blood Pressure Feature";
      case 0x2A4A: return "HID Information";
      case 0x2A4B: return "Report Map";
      case 0x2A4C: return "HID Control Point";
      case 0x2A4D: return "Report";
      case 0x2A4E: return "Protocol Mode";
      case 0x2A4F: return "Scan Interval Window";
      case 0x2A50: return "PnP ID";
      case 0x2A51: return "Glucose Feature";
      case 0x2A52: return "Record Access Control Point";
      case 0x2A53: return "RSC Measurement";
      case 0x2A54: return "RSC Feature";
      case 0x2A55: return "SC Control Point";
      case 0x2A5A: return "Aggregate";
      case 0x2A5B: return "CSC Measurement";
      case 0x2A5C: return "CSC Feature";
      case 0x2A5D: return "Sensor Location";
      case 0x2A5E: return "PLX SpotCheck Measurement";
      case 0x2A5F: return "PLX Continuous Measurement";
      case 0x2A60: return "PLX Features";
      case 0x2A63: return "Cycling Power Measurement";
      case 0x2A64: return "Cycling Power Vector";
      case 0x2A65: return "Cycling Power Feature";
      case 0x2A66: return "Cycling Power Control Point";
      case 0x2A67: return "Location and Speed";
      case 0x2A68: return "Navigation";
      case 0x2A69: return "Position Quality";
      case 0x2A6A: return "LN Feature";
      case 0x2A6B: return "LN Control Point";
      case 0x2A6C: return "Elevation";
      case 0x2A6D: return "Pressure";
      case 0x2A6E: return "Temperature";
      case 0x2A6F: return "Humidity";
      case 0x2A70: return "True Wind Speed";
      case 0x2A71: return "True Wind Direction";
      case 0x2A72: return "Apparent Wind Speed";
      case 0x2A73: return "Apparent Wind Direction";
      case 0x2A74: return "Gust Factor";
      case 0x2A75: return "Pollen Concentration";
      case 0x2A76: return "UV Index";
      case 0x2A77: return "Irradiance";
      case 0x2A78: return "Rainfall";
      case 0x2A79: return "Wind Chill";
      case 0x2A7A: return "Heat Index";
      case 0x2A7B: return "Dew Point";
      case 0x2A7D: return "Descriptor Value Changed";
      case 0x2A7E: return "Aerobic Heart Rate Lower Limit";
      case 0x2A7F: return "Aerobic Threshold";
      case 0x2A80: return "Age";
      case 0x2A81: return "Anaerobic Heart Rate Lower Limit";
      case 0x2A82: return "Anaerobic Heart Rate Upper Limit";
      case 0x2A83: return "Anaerobic Threshold";
      case 0x2A84: return "Aerobic Heart Rate Upper Limit";
      case 0x2A85: return "Date of Birth";
      case 0x2A86: return "Date of Threshold Assessment";
      case 0x2A87: return "Email Address";
      case 0x2A88: return "Fat Burn Heart Rate Lower Limit";
      case 0x2A89: return "Fat Burn Heart Rate Upper Limit";
      case 0x2A8A: return "First Name";
      case 0x2A8B: return "Five Zone Heart Rate Limits";
      case 0x2A8C: return "Gender";
      case 0x2A8D: return "Heart Rate Max";
      case 0x2A8E: return "Height";
      case 0x2A8F: return "Hip Circumference";
      case 0x2A90: return "Last Name";
      case 0x2A91: return "Maximum Recommended Heart Rate";
      case 0x2A92: return "Resting Heart Rate";
      case 0x2A93: return "Sport Type for Aerobic and Anaerobic Thresholds";
      case 0x2A94: return "Three Zone Heart Rate Limits";
      case 0x2A95: return "Two Zone Heart Rate Limits";
      case 0x2A96: return "VO2 Max";
      case 0x2A97: return "Waist Circumference";
      case 0x2A98: return "Weight";
      case 0x2A99: return "Database Change Increment";
      case 0x2A9A: return "User Index";
      case 0x2A9B: return "Body Composition Feature";
      case 0x2A9C: return "Body Composition Measurement";
      case 0x2A9D: return "Weight Measurement";
      case 0x2A9E: return "Weight Scale Feature";
      case 0x2A9F: return "User Control Point";
      case 0x2AA0: return "Magnetic Flux Density 2D";
      case 0x2AA1: return "Magnetic Flux Density 3D";
      case 0x2AA2: return "Language";
      case 0x2AA3: return "Barometric Pressure Trend";
      case 0x2AA4: return "Bond Management Control Point";
      case 0x2AA5: return "Bond Management Feature";
      case 0x2AA6: return "Central Address Resolution";
      case 0x2AA7: return "CGM Measurement";
      case 0x2AA8: return "CGM Feature";
      case 0x2AA9: return "CGM Status";
      case 0x2AAA: return "CGM Session Start Time";
      case 0x2AAB: return "CGM Session Run Time";
      case 0x2AAC: return "CGM Specific Ops Control Point";
      case 0x2AAD: return "Indoor Positioning Configuration";
      case 0x2AAE: return "Latitude";
      case 0x2AAF: return "Longitude";
      case 0x2AB0: return "Local North Coordinate";
      case 0x2AB1: return "Local East Coordinate";
      case 0x2AB2: return "Floor Number";
      case 0x2AB3: return "Altitude";
      case 0x2AB4: return "Uncertainty";
      case 0x2AB5: return "Location Name";
      case 0x2AB6: return "URI";
      case 0x2AB7: return "HTTP Headers";
      case 0x2AB8: return "HTTP Status Code";
      case 0x2AB9: return "HTTP Entity Body";
      case 0x2ABA: return "HTTP Control Point";
      case 0x2ABB: return "HTTPS Security";
      case 0x2ABC: return "TDS Control Point";
      case 0x2ABD: return "OTS Feature";
      case 0x2ABE: return "Object Name";
      case 0x2ABF: return "Object Type";
      case 0x2AC0: return "Object Size";
      case 0x2AC1: return "Object FirstCreated";
      case 0x2AC2: return "Object LastModified";
      case 0x2AC3: return "Object ID";
      case 0x2AC4: return "Object Properties";
      case 0x2AC5: return "Object Action Control Point";
      case 0x2AC6: return "Object List Control Point";
      case 0x2AC7: return "Object List Filter";
      case 0x2AC8: return "Object Changed";
      case 0x2AC9: return "Resolvable Private Address Only";
      case 0x2ACC: return "Fitness Machine Feature";
      case 0x2ACD: return "Treadmill Data";
      case 0x2ACE: return "Cross Trainer Data";
      case 0x2ACF: return "Step Climber Data";
      case 0x2AD0: return "Stair Climber Data";
      case 0x2AD1: return "Rower Data";
      case 0x2AD2: return "Indoor Bike Data";
      case 0x2AD3: return "Training Status";
      case 0x2AD4: return "Supported Speed Range";
      case 0x2AD5: return "Supported Inclination Range";
      case 0x2AD6: return "Supported Resistance Level Range";
      case 0x2AD7: return "Supported Heart Rate Range";
      case 0x2AD8: return "Supported Power Range";
      case 0x2AD9: return "Fitness Machine Control Point";
      case 0x2ADA: return "Fitness Machine Status";
      case 0x2ADB: return "Mesh Provisioning Data In";
      case 0x2ADC: return "Mesh Provisioning Data Out";
      case 0x2ADD: return "Mesh Proxy Data In";
      case 0x2ADE: return "Mesh Proxy Data Out";
      case 0x2AE0: return "Average Current";
      case 0x2AE1: return "Average Voltage";
      case 0x2AE2: return "Boolean";
      case 0x2AE3: return "Chromatic Distance from Planckian";
      case 0x2AE4: return "Chromaticity Coordinates";
      case 0x2AE5: return "Chromaticity in CCT and Duv Values";
      case 0x2AE6: return "Chromaticity Tolerance";
      case 0x2AE7: return "CIE 13.31995 Color Rendering Index";
      case 0x2AE8: return "Coefficient";
      case 0x2AE9: return "Correlated Color Temperature";
      case 0x2AEA: return "Count 16";
      case 0x2AEB: return "Count 24";
      case 0x2AEC: return "Country Code";
      case 0x2AED: return "Date UTC";
      case 0x2AEE: return "Electric Current";
      case 0x2AEF: return "Electric Current Range";
      case 0x2AF0: return "Electric Current Specification";
      case 0x2AF1: return "Electric Current Statistics";
      case 0x2AF2: return "Energy";
      case 0x2AF3: return "Energy in a Period of Day";
      case 0x2AF4: return "Event Statistics";
      case 0x2AF5: return "Fixed String 16";
      case 0x2AF6: return "Fixed String 24";
      case 0x2AF7: return "Fixed String 36";
      case 0x2AF8: return "Fixed String 8";
      case 0x2AF9: return "Generic Level";
      case 0x2AFA: return "Global Trade Item Number";
      case 0x2AFB: return "Illuminance";
      case 0x2AFC: return "Luminous Efficacy";
      case 0x2AFD: return "Luminous Energy";
      case 0x2AFE: return "Luminous Exposure";
      case 0x2AFF: return "Luminous Flux";
      case 0x2B00: return "Luminous Flux Range";
      case 0x2B01: return "Luminous Intensity";
      case 0x2B02: return "Mass Flow";
      case 0x2B03: return "Perceived Lightness";
      case 0x2B04: return "Percentage 8";
      case 0x2B05: return "Power";
      case 0x2B06: return "Power Specification";
      case 0x2B07: return "Relative Runtime in a Current Range";
      case 0x2B08: return "Relative Runtime in a Generic Level Range";
      case 0x2B09: return "Relative Value in a Voltage Range";
      case 0x2B0A: return "Relative Value in an Illuminance Range";
      case 0x2B0B: return "Relative Value in a Period of Day";
      case 0x2B0C: return "Relative Value in a Temperature Range";
      case 0x2B0D: return "Temperature 8";
      case 0x2B0E: return "Temperature 8 in a Period of Day";
      case 0x2B0F: return "Temperature 8 Statistics";
      case 0x2B10: return "Temperature Range";
      case 0x2B11: return "Temperature Statistics";
      case 0x2B12: return "Time Decihour 8";
      case 0x2B13: return "Time Exponential 8";
      case 0x2B14: return "Time Hour 24";
      case 0x2B15: return "Time Millisecond 24";
      case 0x2B16: return "Time Second 16";
      case 0x2B17: return "Time Second 8";
      case 0x2B18: return "Voltage";
      case 0x2B19: return "Voltage Specification";
      case 0x2B1A: return "Voltage Statistics";
      case 0x2B1B: return "Volume Flow";
      case 0x2B1C: return "Chromaticity Coordinate";
      case 0x2B1D: return "RC Feature";
      case 0x2B1E: return "RC Settings";
      case 0x2B1F: return "Reconnection Configuration Control Point";
      case 0x2B20: return "IDD Status Changed";
      case 0x2B21: return "IDD Status";
      case 0x2B22: return "IDD Annunciation Status";
      case 0x2B23: return "IDD Features";
      case 0x2B24: return "IDD Status Reader Control Point";
      case 0x2B25: return "IDD Command Control Point";
      case 0x2B26: return "IDD Command Data";
      case 0x2B27: return "IDD Record Access Control Point";
      case 0x2B28: return "IDD History Data";
      case 0x2B29: return "Client Supported Features";
      case 0x2B2A: return "Database Hash";
      case 0x2B2B: return "BSS Control Point";
      case 0x2B2C: return "BSS Response";
      case 0x2B2D: return "Emergency ID";
      case 0x2B2E: return "Emergency Text";
      case 0x2B2F: return "ACS Status";
      case 0x2B30: return "ACS Data In";
      case 0x2B31: return "ACS Data Out Notify";
      case 0x2B32: return "ACS Data Out Indicate";
      case 0x2B33: return "ACS Control Point";
      case 0x2B34: return "Enhanced Blood Pressure Measurement";
      case 0x2B35: return "Enhanced Intermediate Cuff Pressure";
      case 0x2B36: return "Blood Pressure Record";
      case 0x2B37: return "Registered User";
      case 0x2B38: return "BREDR Handover Data";
      case 0x2B39: return "Bluetooth SIG Data";
      case 0x2B3A: return "Server Supported Features";
      case 0x2B3B: return "Physical Activity Monitor Features";
      case 0x2B3C: return "General Activity Instantaneous Data";
      case 0x2B3D: return "General Activity Summary Data";
      case 0x2B3E: return "CardioRespiratory Activity Instantaneous Data";
      case 0x2B3F: return "CardioRespiratory Activity Summary Data";
      case 0x2B40: return "Step Counter Activity Summary Data";
      case 0x2B41: return "Sleep Activity Instantaneous Data";
      case 0x2B42: return "Sleep Activity Summary Data";
      case 0x2B43: return "Physical Activity Monitor Control Point";
      case 0x2B44: return "Activity Current Session";
      case 0x2B45: return "Physical Activity Session Descriptor";
      case 0x2B46: return "Preferred Units";
      case 0x2B47: return "High Resolution Height";
      case 0x2B48: return "Middle Name";
      case 0x2B49: return "Stride Length";
      case 0x2B4A: return "Handedness";
      case 0x2B4B: return "Device Wearing Position";
      case 0x2B4C: return "Four Zone Heart Rate Limits";
      case 0x2B4D: return "High Intensity Exercise Threshold";
      case 0x2B4E: return "Activity Goal";
      case 0x2B4F: return "Sedentary Interval Notification";
      case 0x2B50: return "Caloric Intake";
      case 0x2B51: return "TMAP Role";
      case 0x2B77: return "Audio Input State";
      case 0x2B78: return "Gain Settings Attribute";
      case 0x2B79: return "Audio Input Type";
      case 0x2B7A: return "Audio Input Status";
      case 0x2B7B: return "Audio Input Control Point";
      case 0x2B7C: return "Audio Input Description";
      case 0x2B7D: return "Volume State";
      case 0x2B7E: return "Volume Control Point";
      case 0x2B7F: return "Volume Flags";
      case 0x2B80: return "Volume Offset State";
      case 0x2B81: return "Audio Location";
      case 0x2B82: return "Volume Offset Control Point";
      case 0x2B83: return "Audio Output Description";
      case 0x2B84: return "Set Identity Resolving Key";
      case 0x2B85: return "Coordinated Set Size";
      case 0x2B86: return "Set Member Lock";
      case 0x2B87: return "Set Member Rank";
      case 0x2B89: return "Apparent Energy 32";
      case 0x2B8A: return "Apparent Power";
      case 0x2B8C: return "CO2 Concentration";
      case 0x2B8D: return "Cosine of the Angle";
      case 0x2B8E: return "Device Time Feature";
      case 0x2B8F: return "Device Time Parameters";
      case 0x2B90: return "Device Time";
      case 0x2B91: return "Device Time Control Point";
      case 0x2B92: return "Time Change Log Data";
      case 0x2B93: return "Media Player Name";
      case 0x2B94: return "Media Player Icon Object ID";
      case 0x2B95: return "Media Player Icon URL";
      case 0x2B96: return "Track Changed";
      case 0x2B97: return "Track Title";
      case 0x2B98: return "Track Duration";
      case 0x2B99: return "Track Position";
      case 0x2B9A: return "Playback Speed";
      case 0x2B9B: return "Seeking Speed";
      case 0x2B9C: return "Current Track Segments Object ID";
      case 0x2B9D: return "Current Track Object ID";
      case 0x2B9E: return "Next Track Object ID";
      case 0x2B9F: return "Parent Group Object ID";
      case 0x2BA0: return "Current Group Object ID";
      case 0x2BA1: return "Playing Order";
      case 0x2BA2: return "Playing Orders Supported";
      case 0x2BA3: return "Media State";
      case 0x2BA4: return "Media Control Point";
      case 0x2BA5: return "Media Control Point Opcodes Supported";
      case 0x2BA6: return "Search Results Object ID";
      case 0x2BA7: return "Search Control Point";
      case 0x2BA8: return "Energy 32";
      case 0x2BA9: return "Media Player Icon Object Type";
      case 0x2BAA: return "Track Segments Object Type";
      case 0x2BAB: return "Track Object Type";
      case 0x2BAC: return "Group Object Type";
      case 0x2BAD: return "Constant Tone Extension Enable";
      case 0x2BAE: return "Advertising Constant Tone Extension Minimum Length";
      case 0x2BAF: return "Advertising Constant Tone Extension Minimum Transmit Count";
      case 0x2BB0: return "Advertising Constant Tone Extension Transmit Duration";
      case 0x2BB1: return "Advertising Constant Tone Extension Interval";
      case 0x2BB2: return "Advertising Constant Tone Extension PHY";
      case 0x2BB3: return "Bearer Provider Name";
      case 0x2BB4: return "Bearer UCI";
      case 0x2BB5: return "Bearer Technology";
      case 0x2BB6: return "Bearer URI Schemes Supported List";
      case 0x2BB7: return "Bearer Signal Strength";
      case 0x2BB8: return "Bearer Signal Strength Reporting Interval";
      case 0x2BB9: return "Bearer List Current Calls";
      case 0x2BBA: return "Content Control ID";
      case 0x2BBB: return "Status Flags";
      case 0x2BBC: return "Incoming Call Target Bearer URI";
      case 0x2BBD: return "Call State";
      case 0x2BBE: return "Call Control Point";
      case 0x2BBF: return "Call Control Point Optional Opcodes";
      case 0x2BC0: return "Termination Reason";
      case 0x2BC1: return "Incoming Call";
      case 0x2BC2: return "Call Friendly Name";
      case 0x2BC3: return "Mute";
      case 0x2BC4: return "Sink ASE";
      case 0x2BC5: return "Source ASE";
      case 0x2BC6: return "ASE Control Point";
      case 0x2BC7: return "Broadcast Audio Scan Control Point";
      case 0x2BC8: return "Broadcast Receive State";
      case 0x2BC9: return "Sink PAC";
      case 0x2BCA: return "Sink Audio Locations";
      case 0x2BCB: return "Source PAC";
      case 0x2BCC: return "Source Audio Locations";
      case 0x2BCD: return "Available Audio Contexts";
      case 0x2BCE: return "Supported Audio Contexts";
      case 0x2BCF: return "Ammonia Concentration";
      case 0x2BD0: return "Carbon Monoxide Concentration";
      case 0x2BD1: return "Methane Concentration";
      case 0x2BD2: return "Nitrogen Dioxide Concentration";
      case 0x2BD3: return "NonMethane Volatile Organic Compounds Concentration";
      case 0x2BD4: return "Ozone Concentration";
      case 0x2BD5: return "Particulate Matter PM1 Concentration";
      case 0x2BD6: return "Particulate Matter PM2.5 Concentration";
      case 0x2BD7: return "Particulate Matter PM10 Concentration";
      case 0x2BD8: return "Sulfur Dioxide Concentration";
      case 0x2BD9: return "Sulfur Hexafluoride Concentration";
      case 0x2BDA: return "Hearing Aid Features";
      case 0x2BDB: return "Hearing Aid Preset Control Point";
      case 0x2BDC: return "Active Preset Index";
      case 0x2BDE: return "Fixed String 64";
      case 0x2BDF: return "High Temperature";
      case 0x2BE0: return "High Voltage";
      case 0x2BE1: return "Light Distribution";
      case 0x2BE2: return "Light Output";
      case 0x2BE3: return "Light Source Type";
      case 0x2BE4: return "Noise";
      case 0x2BE5: return "Relative Runtime in a Correlated Color Temperature Range";
      case 0x2BE6: return "Time Second 32";
      case 0x2BE7: return "VOC Concentration";
      case 0x2BE8: return "Voltage Frequency";
      case 0x2BE9: return "Battery Critical Status";
      case 0x2BEA: return "Battery Health Status";
      case 0x2BEB: return "Battery Health Information";
      case 0x2BEC: return "Battery Information ";
      case 0x2BED: return "Battery Level Status ";
      case 0x2BEE: return "Battery Time Status ";
      case 0x2BEF: return "Estimated Service Date ";
      case 0x2BF0: return "Battery Energy Status ";
    }
  }
#endif
  return nullptr;
}

void exploreService(BLEService service) {
  // print the UUID of the service
  Serial.print("Service ");
  const char *uuid_str = service.uuid();
  Serial.print(uuid_str);

  const char *uuid_name = MapUUIDToName(uuid_str);
  if (uuid_name) {
    Serial.print("\t//");
    Serial.print(uuid_name);
  }
  Serial.println();



  // loop the characteristics of the service and explore each
  for (int i = 0; i < service.characteristicCount(); i++) {
    BLECharacteristic characteristic = service.characteristic(i);

    exploreCharacteristic(characteristic);
  }
}

void exploreCharacteristic(BLECharacteristic characteristic) {
  // print the UUID and properties of the characteristic
  Serial.print("\tCharacteristic ");
  const char *uuid_str = characteristic.uuid();
  Serial.print(uuid_str);
  Serial.print(", properties 0x");
  Serial.print(characteristic.properties(), HEX);

  // check if the characteristic is readable
  if (characteristic.canRead()) {
    // read the characteristic value
    characteristic.read();

    if (characteristic.valueLength() > 0) {
      // print out the value of the characteristic
      Serial.print(", value 0x");
      printData(characteristic.value(), characteristic.valueLength());
    }
  }
  const char *uuid_name = MapUUIDToName(uuid_str);
  if (uuid_name) {
    Serial.print("\t//");
    Serial.print(uuid_name);
  }
  Serial.println();

  // loop the descriptors of the characteristic and explore each
  for (int i = 0; i < characteristic.descriptorCount(); i++) {
    BLEDescriptor descriptor = characteristic.descriptor(i);

    exploreDescriptor(descriptor);
  }
}


void exploreDescriptor(BLEDescriptor descriptor) {
  // print the UUID of the descriptor
  Serial.print("\t\tDescriptor ");
  const char *uuid_str = descriptor.uuid();
  Serial.print(uuid_str);

  // read the descriptor value
  descriptor.read();

  // print out the value of the descriptor
  Serial.print(", value 0x");
  printData(descriptor.value(), descriptor.valueLength());

  const char *uuid_name = MapUUIDToName(uuid_str);
  if (uuid_name) {
    Serial.print("\t//");
    Serial.print(uuid_name);
  }
  Serial.println();
}

void printData(const unsigned char data[], int length) {
  for (int i = 0; i < length; i++) {
    unsigned char b = data[i];

    if (b < 16) {
      Serial.print("0");
    }

    Serial.print(b, HEX);
  }
}
