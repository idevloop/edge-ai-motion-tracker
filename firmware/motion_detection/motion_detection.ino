#include <Wire.h>
#include <U8g2lib.h>
#include <LSM6DS3.h>
#include <Run_inferencing.h>  // Edge Impulse model header

/* Constant defines */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f
#define STANDING_THRESHOLD  0.5f  // Threshold for detecting standing (low acceleration)
#define RUNNING_THRESHOLD   3.0f  // Threshold for detecting running (high acceleration)

// I2C addresses and initialization for XIAO nRF52840
U8X8_SSD1306_64X48_ER_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); // OLED display

// IMU initialization (e.g., LSM6DS3 IMU)
LSM6DS3 myIMU(I2C_MODE, 0x6A);

/* Pin setup */
const int RED_ledPin =  11;   // Adjust pin based on XIAO nRF52840
const int BLUE_ledPin = 12;
const int GREEN_ledPin = 13;

void setup() {
    Serial.begin(115200);
    u8x8.begin();
    Serial.println("Edge Impulse Inferencing Demo - nRF52840");

    // Initialize the IMU
    if (!myIMU.begin()) {
        Serial.println("Failed to initialize IMU!");
    } else {
        Serial.println("IMU initialized");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        Serial.println("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (3 sensor axes)");
        return;
    }

    // Pin initialization
    pinMode(RED_ledPin, OUTPUT);
    pinMode(BLUE_ledPin, OUTPUT);
    pinMode(GREEN_ledPin, OUTPUT);
}

float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

void loop() {
    uint8_t buf1[64] = "idle";
    uint8_t buf2[64] = "running";
    uint8_t buf3[64] = "standing";

    u8x8.clear();
    u8x8.setFont(u8g2_font_ncenB08_tr);

    Serial.println("\nStarting inferencing in 2 seconds...");
    delay(2000);

    Serial.println("Sampling...");

    // Buffer to store IMU values
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        buffer[ix] = myIMU.readFloatAccelX();   // X-axis
        buffer[ix+1] = myIMU.readFloatAccelY(); // Y-axis
        buffer[ix+2] = myIMU.readFloatAccelZ(); // Z-axis

        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Calculate acceleration magnitude
    float accelMagnitude = sqrt(buffer[0] * buffer[0] + buffer[1] * buffer[1] + buffer[2] * buffer[2]);

    // Convert buffer into signal
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        Serial.printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run classifier
    ei_impulse_result_t result = { 0 };
    err = run_classifier(&signal, &result, false);
    if (err != EI_IMPULSE_OK) {
        Serial.printf("Failed to run classifier (%d)\n", err);
        return;
    }

    // Print predictions
    Serial.printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n", 
                  result.timing.dsp, result.timing.classification, result.timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
    }

    // Detect motion and light up LEDs
    if (accelMagnitude < STANDING_THRESHOLD) {
        // Standing detected
        digitalWrite(RED_ledPin, HIGH);  // Red LED for standing
        digitalWrite(BLUE_ledPin, LOW);
        digitalWrite(GREEN_ledPin, LOW);
        u8x8.drawString(2, 3, "Standing");
        //Serial.println("Standing detected.");
    } 
    else if (accelMagnitude > RUNNING_THRESHOLD) {
        // Running detected
        digitalWrite(RED_ledPin, LOW);
        digitalWrite(BLUE_ledPin, HIGH);  // Blue LED for running
        digitalWrite(GREEN_ledPin, LOW);
        u8x8.drawString(2, 3, "Running");
        //Serial.println("Running detected.");
    } 
    else {
        // Idle or light movement detected
        digitalWrite(RED_ledPin, LOW);
        digitalWrite(BLUE_ledPin, LOW);
        digitalWrite(GREEN_ledPin, HIGH);  // Green LED for idle or light movement
        u8x8.drawString(2, 3, "Idle");
        //Serial.println("Idle movement detected.");
    }

    // Refresh display
    u8x8.refreshDisplay();
    delay(2000);
}
 