#pragma once

#include <Arduino.h>
#include <math.h>

class ChargerController
{
public:

    enum class State
    {
        Idle,
        Calibrating,
        Ready,
        Regulating,
        Error
    };

    // Startet die Kalibrierung
    bool startCalibration(
        float maxBatteryCurrent,
        float maxChargerCurrent,
        uint8_t maxPossibleChargers
    );

    // Regelmäßig aus der Hauptschleife aufrufen
    void update(uint32_t nowMs, float batteryCurrent);

    // Startet die eigentliche Regelung nach erfolgreicher Kalibrierung
    bool startRegulation();

    // Beendet die Regelung und setzt den Sollwert auf 0
    void stop();

    State getState() const;
    float getChargerCurrent() const;
    float getChargerFactor() const;

private:

    static constexpr uint8_t NUM_POINTS = 4;

    static constexpr uint32_t SETTLE_TIME = 1500;
    static constexpr uint32_t SAMPLE_TIME = 1000;
    static constexpr uint32_t SAMPLE_INTERVAL = 100;

    static constexpr float MIN_CALIBRATION_CURRENT = 0.4f;

    // PI-Regler
    static constexpr float KP = 0.5f;
    static constexpr float KI = 0.05f;

    State state = State::Idle;

    float maxBatteryCurrent = 0.0f;
    float maxChargerCurrent = 0.0f;
    float maxCalibrationCurrent = 0.0f;

    float chargerFactor = 0.0f;
    float chargerCommand = 0.0f;

    float targetBatteryCurrent = 0.0f;

    float setpoints[NUM_POINTS] = {};
    float measurements[NUM_POINTS] = {};

    uint8_t currentPoint = 0;

    float sampleSum = 0.0f;
    uint16_t sampleCount = 0;

    float integral = 0.0f;

    uint32_t stateTimestamp = 0;
    uint32_t lastSampleTimestamp = 0;
    uint32_t lastControlTimestamp = 0;

    void startMeasurement(uint32_t nowMs);
    void finishMeasurement(uint32_t nowMs);
    bool calculateFactor();

    void updateRegulation(
        uint32_t nowMs,
        float batteryCurrent
    );

    float limitAndRound(float current) const;
};
