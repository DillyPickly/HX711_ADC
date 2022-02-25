/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

#include <HX711_ADC.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "helper.h"
#include "string.h"
#include <algorithm> // std::sort

HX711_ADC::HX711_ADC(uint8_t dout, uint8_t sck) // constructor
{
	doutPin = dout;
	sckPin = sck;
}

void HX711_ADC::setGain(uint8_t gain) // value should be 32, 64 or 128*
{
	if (gain < 64)
		GAIN = 2; // 32, channel B
	else if (gain < 128)
		GAIN = 3; // 64, channel A
	else
		GAIN = 1; // 128, channel A
}

// set pinMode, HX711 gain and power up the HX711
void HX711_ADC::begin()
{
	gpio_config_t cfg = {
		.pin_bit_mask = static_cast<uint64_t>(1) << sckPin,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_DISABLE};
	gpio_config(&cfg);
	cfg.pin_bit_mask = static_cast<uint64_t>(1) << doutPin;
	cfg.mode = GPIO_MODE_INPUT;
	gpio_config(&cfg);
	setGain(128);
	powerUp();
}

// set pinMode, HX711 selected gain and power up the HX711
void HX711_ADC::begin(uint8_t gain)
{
	gpio_config_t cfg = {
		.pin_bit_mask = static_cast<uint64_t>(1) << sckPin,
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_ENABLE,
		.intr_type = GPIO_INTR_DISABLE};
	gpio_config(&cfg);
	cfg.pin_bit_mask = static_cast<uint64_t>(1) << doutPin;
	cfg.mode = GPIO_MODE_INPUT;
	gpio_config(&cfg);
	setGain(gain);
	powerUp();
}

/*  start(t):
 *	will do conversions continuously for 't' +400 milliseconds (400ms is min. settling time at 10SPS).
 *   Running this for 1-5s in setup() - before tare() seems to improve the tare accuracy */
void HX711_ADC::start(unsigned long t)
{
	t += 400;
	lastDoutLowTime = millis();
	while (millis() < t)
	{
		update();
		yield();
	}
	tare();
	tareStatus = 0;
}

/*  start(t, dotare) with selectable tare:
 *	will do conversions continuously for 't' +400 milliseconds (400ms is min. settling time at 10SPS).
 *   Running this for 1-5s in setup() - before tare() seems to improve the tare accuracy. */
void HX711_ADC::start(unsigned long t, bool dotare)
{
	t += 400;
	lastDoutLowTime = millis();
	while (millis() < t)
	{
		update();
		yield();
	}
	if (dotare)
	{
		tare();
		tareStatus = 0;
	}
}

/*  startMultiple(t): use this if you have more than one load cell and you want to do tare and stabilization simultaneously.
 *	Will do conversions continuously for 't' +400 milliseconds (400ms is min. settling time at 10SPS).
 *   Running this for 1-5s in setup() - before tare() seems to improve the tare accuracy */
int HX711_ADC::startMultiple(unsigned long t)
{
	tareTimeoutFlag = 0;
	lastDoutLowTime = millis();
	if (startStatus == 0)
	{
		if (isFirst)
		{
			startMultipleTimeStamp = millis();
			if (t < 400)
			{
				startMultipleWaitTime = t + 400; // min time for HX711 to be stable
			}
			else
			{
				startMultipleWaitTime = t;
			}
			isFirst = 0;
		}
		if ((millis() - startMultipleTimeStamp) < startMultipleWaitTime)
		{
			update(); // do conversions during stabilization time
			yield();
			return 0;
		}
		else
		{ // do tare after stabilization time is up
			static unsigned long timeout = millis() + tareTimeOut;
			doTare = 1;
			update();
			if (convRslt == 2)
			{
				doTare = 0;
				convRslt = 0;
				startStatus = 1;
			}
			if (!tareTimeoutDisable)
			{
				if (millis() > timeout)
				{
					tareTimeoutFlag = 1;
					return 1; // Prevent endless loop if no HX711 is connected
				}
			}
		}
	}
	return startStatus;
}

/*  startMultiple(t, dotare) with selectable tare:
 *	use this if you have more than one load cell and you want to (do tare and) stabilization simultaneously.
 *	Will do conversions continuously for 't' +400 milliseconds (400ms is min. settling time at 10SPS).
 *   Running this for 1-5s in setup() - before tare() seems to improve the tare accuracy */
int HX711_ADC::startMultiple(unsigned long t, bool dotare)
{
	tareTimeoutFlag = 0;
	lastDoutLowTime = millis();
	if (startStatus == 0)
	{
		if (isFirst)
		{
			startMultipleTimeStamp = millis();
			if (t < 400)
			{
				startMultipleWaitTime = t + 400; // min time for HX711 to be stable
			}
			else
			{
				startMultipleWaitTime = t;
			}
			isFirst = 0;
		}
		if ((millis() - startMultipleTimeStamp) < startMultipleWaitTime)
		{
			update(); // do conversions during stabilization time
			yield();
			return 0;
		}
		else
		{ // do tare after stabilization time is up
			if (dotare)
			{
				static unsigned long timeout = millis() + tareTimeOut;
				doTare = 1;
				update();
				if (convRslt == 2)
				{
					doTare = 0;
					convRslt = 0;
					startStatus = 1;
				}
				if (!tareTimeoutDisable)
				{
					if (millis() > timeout)
					{
						tareTimeoutFlag = 1;
						return 1; // Prevent endless loop if no HX711 is connected
					}
				}
			}
			else
				return 1;
		}
	}
	return startStatus;
}

// zero the scale, wait for tare to finnish (blocking)
void HX711_ADC::tare()
{
	uint8_t rdy = 0;
	doTare = 1;
	tareTimes = 0;
	tareTimeoutFlag = 0;
	unsigned long timeout = millis() + tareTimeOut;
	while (rdy != 2)
	{
		rdy = update();
		if (!tareTimeoutDisable)
		{
			if (millis() > timeout)
			{
				tareTimeoutFlag = 1;
				break; // Prevent endless loop if no HX711 is connected
			}
		}
		yield();
	}
}

// zero the scale, initiate the tare operation to run in the background (non-blocking)
void HX711_ADC::tareNoDelay()
{
	doTare = 1;
	tareTimes = 0;
	tareStatus = 0;
}

// set new calibration factor, raw data is divided by this value to convert to readable data
void HX711_ADC::setCalFactor(float cal)
{
	calFactor = cal;
	calFactorRecip = 1 / calFactor;
}

// returns 'true' if tareNoDelay() operation is complete
bool HX711_ADC::getTareStatus()
{
	bool t = tareStatus;
	tareStatus = 0;
	return t;
}

// returns the current calibration factor
float HX711_ADC::getCalFactor()
{
	return calFactor;
}

// call the function update() in loop or from ISR
// if conversion is ready; read out 24 bit data and add to dataset, returns 1
// if tare operation is complete, returns 2
// else returns 0
uint8_t HX711_ADC::update()
{

	byte dout = gpio_get_level(gpio_num_t(doutPin)); // check if conversion is ready
	if (!dout)
	{
		conversion24bit();
		lastDoutLowTime = millis();
		signalTimeoutFlag = 0;
	}
	else
	{
		// if (millis() > (lastDoutLowTime + SIGNAL_TIMEOUT))
		if (millis() - lastDoutLowTime > SIGNAL_TIMEOUT)
		{
			signalTimeoutFlag = 1;
		}
		convRslt = 0;
	}
	return convRslt;
}

float HX711_ADC::getData() // return fresh data from the moving average dataset
{
	long data = 0;
#if USE_MEDIAN_FILTER
	lastSmoothedData = smoothedDataMedian();
#else
	lastSmoothedData = smoothedData();
#endif
	data = lastSmoothedData - tareOffset;
	float x = (float)data * calFactorRecip;
	return x;
}

bool HX711_ADC::isStable()
{
	return stable;
}

long HX711_ADC::smoothedDataMedian()
{
	size_t num_data_samples = samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE + 1;
	for (uint8_t r = 0; r < (samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE); r++)
	{
		dataSampleSetCopy[r] = dataSampleSet[r];
	}
	// for (uint8_t r = 0; r < (samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE); r++)

	// memcpy(dataSampleSetCopy, dataSampleSet, num_data_samples);
	std::sort(dataSampleSetCopy, dataSampleSetCopy + num_data_samples);

	uint8_t mid_point_index = num_data_samples >> 1;
	uint8_t plus_minus_30 = num_data_samples * 3 / 10;
	long stable_range = 0.2 * calFactor;
	long data_range = abs(dataSampleSetCopy[mid_point_index + plus_minus_30] - dataSampleSetCopy[mid_point_index - plus_minus_30]);

	// now we check if the reading is stable
	if (data_range > stable_range)
	{
		stable = false;
	}
	else
	{
		stable = true;
	}

	return dataSampleSetCopy[mid_point_index];
}

long HX711_ADC::smoothedData()
{
	long data = 0;
	long L = 0xFFFFFF;
	long H = 0x00;
	for (uint8_t r = 0; r < (samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE); r++)
	{
#if IGN_LOW_SAMPLE
		if (L > dataSampleSet[r])
			L = dataSampleSet[r]; // find lowest value
#endif
#if IGN_HIGH_SAMPLE
		if (H < dataSampleSet[r])
			H = dataSampleSet[r]; // find highest value
#endif
		data += dataSampleSet[r];
	}
#if IGN_LOW_SAMPLE
	data -= L; // remove lowest value
#endif
#if IGN_HIGH_SAMPLE
	data -= H; // remove highest value
#endif
	// return data;
	return (data >> divBit);
}

void HX711_ADC::conversion24bit() // read 24 bit data, store in dataset and start the next conversion
{
	conversionTime = micros() - conversionStartTime;
	conversionStartTime = micros();
	unsigned long data = 0;
	uint8_t dout;
	convRslt = 0;
	if (SCK_DISABLE_INTERRUPTS)
		noInterrupts();
	for (uint8_t i = 0; i < (24 + GAIN); i++)
	{ // read 24 bit data + set gain and start next conversion
		if (SCK_DELAY)
			delayMicroseconds(3); // could be required for faster mcu's, set value in config.h
		gpio_set_level(gpio_num_t(sckPin), 1);
		if (SCK_DELAY)
			delayMicroseconds(3); // could be required for faster mcu's, set value in config.h
		gpio_set_level(gpio_num_t(sckPin), 0);
		if (i < (24))
		{
			dout = gpio_get_level(gpio_num_t(doutPin));
			data = (data << 1) | dout;
		}
	}
	if (SCK_DISABLE_INTERRUPTS)
		interrupts();
	/*
	The HX711 output range is min. 0x800000 and max. 0x7FFFFF (the value rolls over).
	In order to convert the range to min. 0x000000 and max. 0xFFFFFF,
	the 24th bit must be changed from 0 to 1 or from 1 to 0.
	*/
	data = data ^ 0x800000; // flip the 24th bit

	if (data > 0xFFFFFF)
	{
		dataOutOfRange = 1;
		// Serial.println("dataOutOfRange");
	}
	if (reverseVal)
	{
		data = 0xFFFFFF - data;
	}
	if (readIndex == samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE - 1)
	{
		readIndex = 0;
	}
	else
	{
		readIndex++;
	}
	if (data > 0)
	{
		convRslt++;
		dataSampleSet[readIndex] = (long)data;
		if (doTare)
		{
			if (tareTimes < DATA_SET)
			{
				tareTimes++;
			}
			else
			{
#if USE_MEDIAN_FILTER
				tareOffset = smoothedDataMedian();
#else
				tareOffset = smoothedData();
#endif
				tareTimes = 0;
				doTare = 0;
				tareStatus = 1;
				convRslt++;
			}
		}
	}
}

// power down the HX711
void HX711_ADC::powerDown()
{
	gpio_set_level(gpio_num_t(sckPin), 0);
	gpio_set_level(gpio_num_t(sckPin), 1);
}

// power up the HX711
void HX711_ADC::powerUp()
{
	gpio_set_level(gpio_num_t(sckPin), 0);
}

// get the tare offset (raw data value output without the scale "calFactor")
long HX711_ADC::getTareOffset()
{
	return tareOffset;
}

// set new tare offset (raw data value input without the scale "calFactor")
void HX711_ADC::setTareOffset(long newoffset)
{
	tareOffset = newoffset;
}

// for testing and debugging:
// returns current value of dataset readIndex
int HX711_ADC::getReadIndex()
{
	return readIndex;
}

// for testing and debugging:
// returns latest conversion time in millis
float HX711_ADC::getConversionTime()
{
	return conversionTime / 1000.0;
}

// for testing and debugging:
// returns the HX711 conversions ea seconds based on the latest conversion time.
// The HX711 can be set to 10SPS or 80SPS. For general use the recommended setting is 10SPS.
float HX711_ADC::getSPS()
{
	float sps = 1000000.0 / conversionTime;
	return sps;
}

// for testing and debugging:
// returns the tare timeout flag from the last tare operation.
// 0 = no timeout, 1 = timeout
bool HX711_ADC::getTareTimeoutFlag()
{
	return tareTimeoutFlag;
}

void HX711_ADC::disableTareTimeout()
{
	tareTimeoutDisable = 1;
}

long HX711_ADC::getSettlingTime()
{
	long st = getConversionTime() * DATA_SET;
	return st;
}

// overide the number of samples in use
// value is rounded down to the nearest valid value
void HX711_ADC::setSamplesInUse(int samples)
{
	int old_value = samplesInUse;

	if (samples <= SAMPLES)
	{
		if (samples == 0) // reset to the original value
		{
			divBit = divBitCompiled;
		}
		else
		{
			samples >>= 1;
			for (divBit = 0; samples != 0; samples >>= 1, divBit++)
				;
		}
		samplesInUse = 1 << divBit;

		// replace the value of all samples in use with the last conversion value
		if (samplesInUse != old_value)
		{
			for (uint8_t r = 0; r < samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE; r++)
			{
				dataSampleSet[r] = lastSmoothedData;
			}
			readIndex = 0;
		}
	}
}

// returns the current number of samples in use.
int HX711_ADC::getSamplesInUse()
{
	return samplesInUse;
}

// resets index for dataset
void HX711_ADC::resetSamplesIndex()
{
	readIndex = 0;
}

// Fill the whole dataset up with new conversions, i.e. after a reset/restart (this function is blocking once started)
bool HX711_ADC::refreshDataSet()
{
	int s = getSamplesInUse() + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE; // get number of samples in dataset
	resetSamplesIndex();
	while (s > 0)
	{
		update();
		yield();
		if (gpio_get_level(gpio_num_t(doutPin)) == 0)
		{			   // HX711 dout pin is pulled low when a new conversion is ready
			getData(); // add data to the set and start next conversion
			s--;
		}
	}
	return true;
}

// returns 'true' when the whole dataset has been filled up with conversions, i.e. after a reset/restart.
bool HX711_ADC::getDataSetStatus()
{
	bool i = false;
	if (readIndex == samplesInUse + IGN_HIGH_SAMPLE + IGN_LOW_SAMPLE - 1)
	{
		i = true;
	}
	return i;
}

// returns and sets a new calibration value (calFactor) based on a known mass input
float HX711_ADC::getNewCalibration(float known_mass)
{
	float readValue = getData();
	float exist_calFactor = getCalFactor();
	float new_calFactor;
	new_calFactor = (readValue * exist_calFactor) / known_mass;
	setCalFactor(new_calFactor);
	return new_calFactor;
}

// returns 'true' if it takes longer time then 'SIGNAL_TIMEOUT' for the dout pin to go low after a new conversion is started
bool HX711_ADC::getSignalTimeoutFlag()
{
	return signalTimeoutFlag;
}

// reverse the output value (flip positive/negative value)
// tare/zero-offset must be re-set after calling this.
void HX711_ADC::setReverseOutput()
{
	reverseVal = true;
}