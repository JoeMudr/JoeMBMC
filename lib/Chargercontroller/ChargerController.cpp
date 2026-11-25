
#include "ChargerController.h"

bool ChargerController::startCalibration(float maxBatteryCurrent, float maxChargerCurrent, uint8_t maxPossibleChargers)
{
    if (maxBatteryCurrent <= 0.0f ||
        maxChargerCurrent <= 0.0f ||
        maxPossibleChargers == 0)
    {
        state = State::Error;
        return false;
    }

    this->maxBatteryCurrent = maxBatteryCurrent;
    this->maxChargerCurrent = maxChargerCurrent;

    // Sicherheitsbegrenzung während der Kalibrierung.
    // Annahme: Alle Ladegeräte reagieren identisch.
    maxCalibrationCurrent =
        min(maxChargerCurrent,
            maxBatteryCurrent / maxPossibleChargers);

    if (maxCalibrationCurrent < MIN_CALIBRATION_CURRENT)
    {
        state = State::Error;
        return false;
    }

    // Vier gleichmäßig verteilte Messpunkte
    for (uint8_t i = 0; i < NUM_POINTS; i++)
    {
        setpoints[i] =
            maxCalibrationCurrent * (i + 1) / NUM_POINTS;
    }

    currentPoint = 0;
    chargerCommand = 0.0f;
    chargerFactor = 0.0f;
    integral = 0.0f;

    state = State::Calibrating;
    
    startMeasurement(millis());

    return true;
}


void ChargerController::update(
    uint32_t nowMs,
    float batteryCurrent)
{
    switch (state)
    {
        case State::Calibrating:

            // Sicherheitsabschaltung bei Überschreitung
            if (batteryCurrent >= maxBatteryCurrent)
            {
                chargerCommand = 0.0f;
                state = State::Error;
                return;
            }

            if (nowMs - stateTimestamp < SETTLE_TIME)
                return;

            if (nowMs - lastSampleTimestamp >= SAMPLE_INTERVAL)
            {
                sampleSum += batteryCurrent;
                sampleCount++;

                lastSampleTimestamp = nowMs;
            }

            if (nowMs - stateTimestamp >=
                SETTLE_TIME + SAMPLE_TIME)
            {
                finishMeasurement(nowMs);
            }

            break;


        case State::Regulating:

            updateRegulation(nowMs, batteryCurrent);
            break;


        default:
            break;
    }
}


void ChargerController::startMeasurement(uint32_t nowMs)
{
    sampleSum = 0.0f;
    sampleCount = 0;

    chargerCommand = setpoints[currentPoint];

    stateTimestamp = nowMs;
    lastSampleTimestamp = nowMs;
}


void ChargerController::finishMeasurement(uint32_t nowMs)
{
    if (sampleCount == 0)
    {
        state = State::Error;
        chargerCommand = 0.0f;
        return;
    }

    measurements[currentPoint] =
        sampleSum / sampleCount;

    currentPoint++;

    if (currentPoint >= NUM_POINTS)
    {
        chargerCommand = 0.0f;

        if (calculateFactor())
            state = State::Ready;
        else
            state = State::Error;

        return;
    }

    startMeasurement(nowMs);
}


bool ChargerController::calculateFactor()
{
    float meanS = 0.0f;
    float meanI = 0.0f;

    for (uint8_t i = 0; i < NUM_POINTS; i++)
    {
        meanS += setpoints[i];
        meanI += measurements[i];
    }

    meanS /= NUM_POINTS;
    meanI /= NUM_POINTS;

    float numerator = 0.0f;
    float denominator = 0.0f;

    for (uint8_t i = 0; i < NUM_POINTS; i++)
    {
        float ds = setpoints[i] - meanS;
        float di = measurements[i] - meanI;

        numerator += ds * di;
        denominator += ds * ds;
    }

    if (denominator <= 0.0f)
        return false;

    chargerFactor = numerator / denominator;

    // Plausibilitätsprüfung
    if (chargerFactor < 0.5f ||
        chargerFactor > 10.0f)
    {
        chargerFactor = 0.0f;
        return false;
    }

    return true;
}


bool ChargerController::startRegulation()
{
    if (state != State::Ready)
        return false;

    chargerCommand = 0.0f;
    integral = 0.0f;

    state = State::Regulating;

    lastControlTimestamp = millis();

    return true;
}


void ChargerController::updateRegulation(
    uint32_t nowMs,
    float batteryCurrent)
{
    // Harte Stromgrenze
    if (batteryCurrent >= maxBatteryCurrent)
    {
        chargerCommand = 0.0f;
        integral = 0.0f;
        return;
    }

    uint32_t elapsed = nowMs - lastControlTimestamp;

    // Regelung maximal alle 100 ms
    if (elapsed < 100)
        return;

    lastControlTimestamp = nowMs;

    float dt = elapsed / 1000.0f;

    // Sollwert ist 0 A Batteriestrom
    float error = targetBatteryCurrent - batteryCurrent;

    // PI-Regler auf den Gesamtstrom
    float proportional = KP * error;

    float maxTotalCommand =
        chargerFactor * maxChargerCurrent;

    float newIntegral =
        integral + KI * error * dt;

    // Begrenzung des Integrators
    newIntegral = constrain(
        newIntegral,
        -maxTotalCommand,
        maxTotalCommand
    );

    float totalCommand =
        proportional + newIntegral;

    // Begrenzung auf die verfügbare Ladeleistung
    totalCommand = constrain(
        totalCommand,
        0.0f,
        maxTotalCommand
    );

    // Anti-Windup
    if ((totalCommand > 0.0f &&
         totalCommand < maxTotalCommand) ||
        (totalCommand == 0.0f && error > 0.0f) ||
        (totalCommand == maxTotalCommand && error < 0.0f))
    {
        integral = newIntegral;
    }

    // Gesamtstrom auf die einzelnen Ladegeräte verteilen
    chargerCommand =
        limitAndRound(totalCommand / chargerFactor);
}


float ChargerController::limitAndRound(float current) const
{
    current = constrain(current, 0.0f, maxChargerCurrent);

    return roundf(current * 10.0f) / 10.0f;
}


void ChargerController::stop()
{
    chargerCommand = 0.0f;
    integral = 0.0f;
    state = State::Idle;
}


ChargerController::State ChargerController::getState() const
{
    return state;
}


float ChargerController::getChargerCurrent() const
{
    return chargerCommand;
}


float ChargerController::getChargerFactor() const
{
    return chargerFactor;
}
