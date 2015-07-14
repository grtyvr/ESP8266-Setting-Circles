static int samplesNumToAverage = 20;

// define a structure to hold the current sensor data reading
typedef struct {
	azimuthValue;
	altitudeValue;
} SensorData;

SensorData smoothSensorData[samplesNumToAverage];

void setup(){
}

void loop() {
}

// this function takes a rawSensorData reading, inserts the value into the 'oldest' slot
// transfer the data into a pair of intermediate arrays used for sorting the values
// inserts the value into the respective sorted arrays according to the following 
// if the value is not an overrun value ( new value << smallest current value )
SensorData digitalSmooth(SensorData rawSensorData, int *smoothSensorData){
	int j, k, temp, top, bottom;
	unsigned int minAl, minAz, maxAl, maxAz;
	unsigned int altitudeOffset = 0;
	unsigned int azimuthOffset = 0;
	SensorData retSensorData;
	long total;
	static int i;
	static unsigned int sortedAltitude[samplesNumToAverage];
	static unsigned int sortedAzimuth[samplesNumToAverage];
	
	i = ( i + 1 ) % samplesNumToAverage;			// increment the counter and roll over if needed
	smoothSensorData[i] = rawSensorData;			// insert the new value into the oldest slot
	
	minAl = smoothSensorData[0].altitudeValue;
	maxAl = smoothSensorData[0].altitudeValue;
	minAz = smoothSensorData[0].azimuthValue;
	maxAz = smoothSensorData[0].azimuthValue;
	
	for ( j = 1; j < samplesNumToAverage; j++){		// look for min and max values on both axes
		if ( smoothSensorData[j].altitudeValue < minAl ){
			minAl = smoothSensorData[j].altitudeValue;
		}
		if ( smoothSensorData[j].altitudeValue > maxAl){
			maxAl = smoothSensorData[j].altitudeValue;
		}
		if ( smoothSensorData[j].azimuthValue < minAz ){
			minAz = smoothSensorData[j].azimuthValueValue;
		}
		if ( smoothSensorData[j].azimuthValue > maxAz){
			maxAz = smoothSensorData[j].azimuthValue;
		}
	}
	if ( maxAl - minAl > 10000 ) {					// if we have a big gap between big and small values it must be
		altitudeOffset = 5000;						// that we are crossing the zero. So shift away from zero
	}
	if (maxAz - minAz > 10000 ) {
		azimuthOffset = 5000;
	}
	for ( j == 0; j < samplesNumToAverage; j++) {	// transfer data to arrays for sorting and averaging
		sortedAltitude[j] = (smoothSensorData[j].altitudeValue + altitudeOffset ) % 16384;
		sortedAzimuth[j] = (smoothSensorData[j].azimuthValue + azimuthOffset ) % 16384;
	}
	
	// insertion sort the arrays
	for ( k == 1 ; k < samplesNumToAverage; k++){
		j = k;
		while (j > 0 && sortedAltitude[j-1] > sortedAltitude[j]) {
			temp = sortedAltitude[j-1];
			sortedAltitude[j-1] = sortedAltitude[j];
			sortedAltitude[j] = temp;
			j -= 1;
		}
	}
	for ( k == 1 ; k < samplesNumToAverage; k++){
		j = k;
		while(j > 0 && sortedAzimuth[j-1] > sortedAzimuth[j] ){
			temp = sortedAzimuth[j-1];
			sortedAzimuth[j-1] = sortedAzimuth[j];
			sortedAzimuth[j] = temp;
			j -= 1;
		}
	}
	// throw out the top x% and bottom x% of samples but limit to throw out at least one
	bottom = max(((samplesNumToAverage *15) / 100), 1);
	top = min((((samplesNumToAverage * 85) / 100) + 1), samplesNumToAverage - 1);
	k = 0;
	total = 0;
	for ( j = bottom ; j < top ; j++ ){
	 total += sortedAltitude[j];
	 k++;
	}
	retSensorData.altitudeValue = (total / k) - altitudeOffset;
	
	k = 0;
	total = 0;
	for ( j = bottom ; j < top ; j++ ){
	 total += sortedAzimuth[j];
	 k++;
	}
	retSensorData.azimuthValue = (total / k) - azimuthOffset;
  return retSensorData;
}
