/**
 * @file CH101_STM32F407_Demo.ino
 * @brief CH101 Ultrasonic Sensor Demo for STM32F407 Discovery Board
 * @details Demonstrates distance measurement using CH101 sensor with visual LED feedback
 * @author Emad Roshandel
 * @date 2025
 * @hardware STM32F407 Discovery Board, CH101 Ultrasonic Sensor
 * @note Requires bidirectional level shifter for INT pin (e.g., SN74LVC2T45)
 */

// ============================================================================
// LIBRARY INCLUDES
// ============================================================================
#include <Wire.h>                    // I2C communication library
#include <Adafruit_I2CDevice.h>      // Adafruit I2C device abstraction
#include <SoftwareSerial.h>          // Software serial (not used but included)
#include "CH101.h"                   // CH101 ultrasonic sensor driver

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================
// Output format selector:
// 1 = Raw I/Q data (in-phase and quadrature samples)
// 2 = Distance data (range in millimeters)
#define outputformat 2

// CH101 sensor pin definitions
#define INT1 PE6          // Interrupt pin 1 - sensor sends data ready signal
#define INT_DIR PE4       // Interrupt direction control for bidirectional level shifter
#define RstPin PE5        // Reset pin - used to reset the sensor
#define ProgPin PC13      // Programming pin - used during firmware upload

// Create CH101 sensor object with I2C interface and pin assignments
// Parameters: (I2C bus, INT1 pin, INT_DIR pin, Reset pin, Prog pin, use_reset)
CH101 CHx01em(Wire, INT1, INT_DIR, RstPin, ProgPin, true);

// ============================================================================
// LED CONFIGURATION (STM32F407 Discovery Board)
// ============================================================================
#define LED_GREEN  PD12   // Green LED - indicates system ready
#define LED_ORANGE PD13   // Orange LED - indicates data acquisition in progress
#define LED_RED    PD14   // Red LED - indicates startup/error state
#define LED_BLUE   PD15   // Blue LED - indicates sensor initialization

// ============================================================================
// SERIAL PORT CONFIGURATION
// ============================================================================
// Serial port pins (not used in this implementation)
#define RXpin   PA3       // UART RX pin
#define TXpin   PA2       // UART TX pin

// Serial3 configuration for debug output
#define RXpin3  PB11      // UART3 RX pin
#define TXpin3  PD8       // UART3 TX pin
HardwareSerial Serial3(RXpin3, TXpin3);  // Create Serial3 instance

// ============================================================================
// I2C CONFIGURATION
// ============================================================================
#define SDApin PB9        // I2C data line
#define SCLpin PB6        // I2C clock line

// ============================================================================
// LED ARRAY (for future animation features)
// ============================================================================
const uint8_t leds[] = { LED_GREEN, LED_ORANGE, LED_RED, LED_BLUE };
const int numLEDs = sizeof(leds) / sizeof(leds[0]);  // Calculate number of LEDs

// ============================================================================
// SETUP FUNCTION - Runs once at startup
// ============================================================================
void setup() {
  // --------------------------------------------------------------------------
  // Initialize Serial Communication
  // --------------------------------------------------------------------------
  Serial.begin(115200);   // Initialize Serial (USB) at 115200 baud
  Serial3.begin(115200);  // Initialize Serial3 (UART3) at 115200 baud

  // --------------------------------------------------------------------------
  // Initialize LED Pins
  // --------------------------------------------------------------------------
  pinMode(LED_GREEN, OUTPUT);   // Configure green LED as output
  pinMode(LED_ORANGE, OUTPUT);  // Configure orange LED as output
  pinMode(LED_RED, OUTPUT);     // Configure red LED as output
  pinMode(LED_BLUE, OUTPUT);    // Configure blue LED as output
  
  // Turn all LEDs OFF at startup for a clean initial state
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_ORANGE, LOW);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_BLUE, LOW);
  
  // Briefly flash RED LED to indicate system startup
  digitalWrite(LED_RED, HIGH);
  delay(500);                   // Wait 500ms
  digitalWrite(LED_RED, LOW);

  // --------------------------------------------------------------------------
  // Initialize I2C Communication
  // --------------------------------------------------------------------------
  Serial3.println("Check Serial");  // Send test message to Serial3
  
  Wire.setSDA(SDApin);  // Set custom SDA pin for I2C
  Wire.setSCL(SCLpin);  // Set custom SCL pin for I2C
  Wire.begin();         // Initialize I2C bus as master
  
  // --------------------------------------------------------------------------
  // Initialize CH101 Sensor
  // --------------------------------------------------------------------------
  Serial3.println("Now starting normal initialization...");
  
  digitalWrite(LED_BLUE, HIGH);  // Turn on BLUE LED during initialization
  
  // Initialize the CH101 sensor (loads firmware, calibrates, etc.)
  int ret = CHx01em.begin();
  
  // Check if initialization was successful
  if (ret != 0) {
    // Initialization failed - print error information
    Serial3.print("CH101 initialization FAILED with error code: ");
    Serial3.println(ret);
    Serial3.println("\nTroubleshooting steps:");
    Serial3.println("1. Try changing 'true' to 'false' in CH101 constructor");
    Serial3.println("2. Check PROG pin connection (pin 5)");
    Serial3.println("3. Check INT1 pin connection (pin 14)");
    Serial3.println("4. Verify 3.3V power supply is stable");
    Serial3.println("5. Try with a different Arduino board if available");
    
    digitalWrite(LED_BLUE, LOW);  // Turn off blue LED
    
    // Enter infinite error loop with blinking RED LED
    while (1) {
      digitalWrite(LED_RED, HIGH);
      delay(500);
      digitalWrite(LED_RED, LOW);
      delay(500);
    }
  } else {
    // Initialization successful
    Serial3.println("CH101 initialized successfully!");
    CHx01em.print_informations();  // Print sensor information (part number, freq, etc.)
    digitalWrite(LED_BLUE, LOW);    // Turn off blue LED
  }
  
  // --------------------------------------------------------------------------
  // Configure Sensor Operating Mode
  // --------------------------------------------------------------------------
  Serial3.println("\nStarting free run mode...");
  
  // Start sensor in free-running mode
  // Parameters: (max_range_mm, measurement_interval_ms)
  // max_range: Maximum detection range in millimeters
  // interval: Time between measurements (200ms = 5 Hz sample rate)
  ret = CHx01em.free_run(CHx01em.get_max_range(), 200);
  
  // Check if free-run mode started successfully
  if (ret != 0) {
    Serial3.print("CH101 free run FAILED with error code: ");
    Serial3.println(ret);
    
    // Enter infinite error loop with blinking RED LED
    while (1) {
      digitalWrite(LED_RED, HIGH);
      delay(500);
      digitalWrite(LED_RED, LOW);
      delay(500);
    }
  } else {
    // Free-run mode started successfully
    Serial3.println("Free run mode started successfully!");
    CHx01em.print_configuration();  // Print current sensor configuration
  }
  
  Serial3.println("\n=== Ready to collect data ===\n");
  
  digitalWrite(LED_GREEN, HIGH);  // Turn on GREEN LED to indicate ready state
}

// ============================================================================
// LOOP FUNCTION - Runs continuously after setup
// ============================================================================
void loop() {
  // Turn on ORANGE LED to indicate data acquisition is in progress
  digitalWrite(LED_ORANGE, HIGH);
  
  // Select output format based on the defined format at the top of the file
  switch(outputformat) {
    
    // ------------------------------------------------------------------------
    // Case 1: Raw I/Q Data Output
    // ------------------------------------------------------------------------
    case 1: {
      // Declare buffer for raw sensor data
      ch_iq_sample_t raw_data[CH101_MAX_NUM_SAMPLES];  // Array to hold I/Q samples
      uint16_t nb_samples;  // Variable to store the number of samples received
      
      // Check if new data is available from the sensor
      if (CHx01em.data_ready()) {
        // Retrieve raw I/Q data from sensor
        CHx01em.get_iq_data(raw_data, nb_samples);
        
        // Print header with sample count
        Serial3.print("CH101 Raw Data (");
        Serial3.print(nb_samples);
        Serial3.println(" samples):");
        
        // Loop through all samples and print I/Q values
        for (int count = 0; count < nb_samples; count++) {
          Serial3.print("q=");                  // Quadrature component
          Serial3.print(raw_data[count].q);
          Serial3.print(",i=");                 // In-phase component
          Serial3.println(raw_data[count].i);
        }
        Serial3.println("---");  // Print separator
      }
      break;
    }
    
    // ------------------------------------------------------------------------
    // Case 2: Distance Data Output (Default)
    // ------------------------------------------------------------------------
    case 2: {
      // Check if new measurement is available
      if (CHx01em.data_ready()) {
        // Get calculated range in millimeters
        float range = CHx01em.get_range();
        
        // Print range value
        Serial3.print("Range(mm): ");
        Serial3.println(range);
      }
      break;
    }
    
    // ------------------------------------------------------------------------
    // Default Case: Invalid Format
    // ------------------------------------------------------------------------
    default:
      Serial3.println("Invalid output format!");
      break;
  }
  
  // Turn off ORANGE LED to indicate data acquisition complete
  digitalWrite(LED_ORANGE, LOW);
  
  // Wait 100ms before next loop iteration
  // This provides a brief pause between data reads
  delay(100);
}
