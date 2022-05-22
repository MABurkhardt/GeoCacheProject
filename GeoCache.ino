/******************************************************************************
GeoCache Hunt Project (GeoCache.ino)
This is skeleton code provided as a project development guideline only.  You
are not required to follow this coding structure.  You are free to implement
your project however you wish.  Be sure you compile in "RELEASE" mode.
Complete the team information below before submitting code for grading.
Team Number: ?
Team Members: ?
NOTES:
You only have 32k of program space and 2k of data space.  You must
use your program and data space wisely and sparingly.  You must also be
very conscious to properly configure the digital pin usage of the boards,
else weird things will happen.
The Arduino GCC sprintf() does not support printing floats or doubles.  You should
consider using sprintf(), dtostrf(), strtok() and strtod() for message string
parsing and converting between floats and strings.
The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need convert it to Decimal Degrees format (DDD.DDDD).  The switch on the
GPS Shield must be set to the "Soft Serial" position, else you will not receive
any GPS messages.
*******************************************************************************
Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must parse the message to obtain the parameters required for
the GeoCache project.  GPS provides coordinates in Degrees Minutes (DDDMM.MMMM).
The coordinates in the following GPRMC sample message, after converting to Decimal
Degrees format(DDD.DDDDDD) is latitude(23.118757) and longitude(120.274060).  By
the way, this coordinate is GlobalTop Technology in Taiwan, who designed and
manufactured the GPS Chip.
"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C/r/n"
$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm) 0-90
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm) 0-180
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
/r/n            // return and newline
The results below are calculated from above GPS GPRMC message
and the GEOLAT0/GEOLON0 tree as target.  Your results should be
nearly identical, if not exactly the same.
Results of converting GPS LAT or LON string to a float:
LAT_2307.1256 = 2307.125488
LON_12016.4438 = 12016.443359
Results of executing the following functions:
degMin2DecDeg() LAT_2307.1256_N = 23.118757 decimal degrees
degMin2DecDeg() LON_12016.4438_E = 120.274055 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999652 degrees
Results for adjusting for relative bearing towards tree = 217.519650 degrees
******************************************************************************/

/*
Configuration settings.
These defines makes it easy for you to enable/disable certain
code during the development and debugging cycle of this project.
There may not be sufficient room in the PROGRAM or DATA memory to
enable all these libraries at the same time.  You must have NEO_ON,
GPS_ON and SDC_ON during the actual GeoCache Flag Hunt on Finals Day.
*/
#define NEO_ON 0		// NeoPixel Shield (0=OFF, 1=ON)
#define LOG_ON 1		// Serial Terminal Logging (0=OFF, 1=ON)
#define SDC_ON 0		// Secure Digital Card (0=OFF, 1=ON)
#define GPS_ON 0		// 0 = simulated GPS message, 1 = actual GPS message

// define pin usage
#define NEO_TX	6		// NEO transmit
#define GPS_TX	7		// GPS transmit
#define GPS_RX	8		// GPS receive

#define BUT_PIN 2

#define GPS_BUFSIZ	96	// max size of GPS char buffer

// global variables
uint8_t target = 0;		// target number
float heading = 0.0;	// target heading
float distance = 0.0;	// target distance

#if GPS_ON
#include <SoftwareSerial.h>
SoftwareSerial gps(GPS_RX, GPS_TX);
#endif

#if NEO_ON
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, NEO_TX, NEO_GRB + NEO_KHZ800);

uint32_t targetColor = Adafruit_NeoPixel::Color(0, 0, 255);;

static struct Compass
{
	// Number of circle segments for our LED-compass
	int arcs = 16;
	// angle length of each quadrant
	float arclength = 360.0f / (float)arcs;
	// gives an error-margin for each arc
	float offset = arclength / 2.0f;

	/*
	Note:
	 dividing a circle (360 degrees) in 16 arcs
	 makes each one 22.5 degrees and gives an
	 offset of 11.25
	*/
};

/*
Brighteness control Gamma correction
https://learn.adafruit.com/led-tricks-gamma-correction/the-issue
Compiler directive "PROGMEM" puts array in program code memory space
*/
const uint8_t PROGMEM gamma[] =
{
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
	1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
	2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
	5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
	10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
	17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
	25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
	37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
	51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
	69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
	90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
	115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
	144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
	177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
	215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255
};
#endif

#if SDC_ON
#include <SD.h>
File myFile;
#endif

/*
Following is a Decimal Degrees formatted waypoint for the large tree
in the parking lot just outside the front entrance of FS3B-116.
On GeoCache day, you will be given waypoints in Decimal Degrees format for 4x
flags located on Full Sail campus.
*/
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437
#define GEOLAT1 2.222222
#define GEOLON1 -2.222222
#define GEOLAT2 3.333333
#define GEOLON2 -3.333333
#define GEOLAT3 4.444444
#define GEOLON3 -4.444444

//TODO create array to hold 4x coordinates of flags
float coordArr[8] = { GEOLAT0, GEOLON0, GEOLAT1, GEOLON1, GEOLAT2, GEOLON2, GEOLAT3, GEOLON3 };

#if GPS_ON
/*
These are GPS command messages (only a few are used).
*/
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#endif

/*************************************************
**** GEO FUNCTIONS - BEGIN ***********************
*************************************************/

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)
float degMin2DecDeg(char *cind, char *ccor)
Input:
	cind = char string pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator
	ccor = char string pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate
Return:
	Decimal degrees coordinate.
**************************************************/
float degMin2DecDeg(char* cind, char* ccor)
{
	//TODO convert degrees minutes to decimal degrees
	float degrees = 0.0;
	float ccorStore = strtod(ccor, NULL);
	float degStore = 0;
	float minutesStore = 0;

	degrees = int(ccorStore / 100);
	degStore = degrees * 100;
	minutesStore = ccorStore - degStore;
	degrees = degrees + (minutesStore / 60);

	if (cind == "W" || cind == "S")
	{
		degrees = degrees * -1;
	}

#if LOG_ON
	Serial.print("degMin2DecDeg() returned: ");
	Serial.println(degrees, 6);
#endif

	return(degrees);
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.
float calcDistance(float flat1, float flon1, float flat2, float flon2)
EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile
Input:
	flat1, flon1 = GPS latitude and longitude coordinate in decimal degrees
	flat2, flon2 = Target latitude and longitude coordinate in decimal degrees
Return:
	distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	//TODO calculated distance to target
	float distance = 0.0;
	float R = 20903520;		//earth's radius in feet

	float lat1 = flat1 * PI / 180;	//first lat in radians
	float lat2 = flat2 * PI / 180;	//second lat in radians
	float deltaLat = (flat2 - flat1) * PI / 180;	//change in lat in radians
	float deltaLon = (flon2 - flon1) * PI / 180;	//change in lon in radians

	float x = sin(deltaLat / 2) * sin(deltaLat / 2) + cos(lat1) * cos(lat2)
		* sin(deltaLon / 2) * sin(deltaLon / 2);
	float arcLeng = 2 * atan2(sqrt(x), sqrt(1 - x));

	distance = R * arcLeng;

#if LOG_ON
	Serial.print("calcDistance() returned: ");
	Serial.println(distance, 6);
#endif

	return(distance);
}

/**************************************************
Calculate Great Circle Bearing between two coordinates
float calcBearing(float flat1, float flon1, float flat2, float flon2)
Input:
	flat1, flon1 = gps latitude and longitude coordinate in decimal degrees
	flat2, flon2 = target latitude and longitude coordinate in decimal degrees
Return:
	angle in decimal degrees from magnetic north
NOTE: atan2() returns range of -pi/2 to +pi/2)
**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	//TODO calculate bearing to target
	float bearing = 0.0;
	//lat & lon in radians
	float gpsLat = radians(flat1);
	float targetLat = radians(flat2);
	float deltaLat = radians(flat2 - flat1);
	float deltaLon = radians(flon2 - flon1);

	float y = sin(deltaLon) * cos(targetLat);
	float x = (cos(gpsLat) * sin(targetLat)) - (sin(gpsLat) * cos(targetLat) * cos(deltaLon));

	float ang = atan2(y, x);
	bearing = degrees(ang);
	bearing = fmod((bearing + 360), 360);
	//bearing = ang;

#if LOG_ON
	Serial.print("calcBearing() returned: ");
	Serial.println(bearing, 6);
#endif

	return(bearing);
}

/*************************************************
**** GEO FUNCTIONS - END**************************
*************************************************/

#if NEO_ON
void setNeoPixel(uint8_t target, float heading, float distance, uint8_t potentiometer)
{
	// wipes previously set LED's
	strip.clear();

	int8_t compassLED = (heading + Compass().offset) / Compass().arclength;

	switch (target)
	{
	case 0:
	{
		targetColor = strip.Color(255, 255, 0);
		strip.setPixelColor(compassLED, targetColor);


		break;
	}
	case 1:
	{
		targetColor = strip.Color(255, 0, 255);
		break;
	}
	case 2:
	{
		targetColor = strip.Color(0, 255, 255);
		break;
	}
	case 3:
	{
		targetColor = strip.Color(255, 255, 255);
		break;
	};
	default:
	{
		targetColor = strip.Color(255, 0, 0);
		break;
	}
	}

	if (BUT_PIN == LOW)
	{
	}

	/*
		TODO display of target, heading, distance
	*/
}
#endif	// NEO_ON

#if GPS_ON
/*
Get valid GPS message.
char* getGpsMessage(void)
Side affects:
Message is placed in local static char buffer.
Input:
none
Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received
*/
char* getGpsMessage(void)
{
	bool rv = false;
	static uint8_t x = 0;
	static char cstr[GPS_BUFSIZ];

	// get nmea string
	while (gps.peek() != -1)
	{
		// reset or bound cstr
		if (x == 0) memset(cstr, 0, sizeof(cstr));
		else if (x >= (GPS_BUFSIZ - 1)) x = 0;

		// read next char
		cstr[x] = gps.read();

		// toss invalid or undesired message
		if ((x >= 3) && (cstr[0] != '$') && (cstr[3] != 'R'))
		{
			x = 0;
			break;
		}

		// if end of message received (sequence is \r\n)
		if (cstr[x] == '\n')
		{
			// nul terminate char buffer (before \r\n)
			cstr[x - 1] = 0;

			// if checksum not found
			if (cstr[x - 4] != '*')
			{
				x = 0;
				break;
			}

			// convert hex checksum to binary
			uint8_t isum = strtol(&cstr[x - 3], NULL, 16);

			// reverse checksum
			for (uint8_t y = 1; y < (x - 4); y++) isum ^= cstr[y];

			// if invalid checksum
			if (isum != 0)
			{
				x = 0;
				break;
			}

			// else valid message
			rv = true;
			x = 0;
			break;
		}

		// increment buffer position
		else x++;

		// software serial must breath, else miss incoming characters
		delay(1);
	}

	if (rv) return(cstr);
	else return(nullptr);
}

#else

/*
Get simulated GPS message (same as message described at top of this *.ino file)
char* getGpsMessage(void)
Side affects:
Message is place in local static char buffer
Input:
none
Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received
*/
char* getGpsMessage(void)
{
	static char cstr[GPS_BUFSIZ];
	static uint32_t timestamp = 0;

	uint32_t timenow = millis();

	// provide message every second
	if (timestamp >= timenow) return(nullptr);

	memcpy(cstr, "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C", sizeof(cstr));

	timestamp = timenow + 1000;

	return(cstr);
}
#endif

void setup(void)
{
#if LOG_ON
	// init serial interface
	Serial.begin(115200);
#endif	

#if NEO_ON
	// TODO init NeoPixel Shield
	strip.begin();
	strip.show();
	strip.setBrightness(10);
#endif	

#if SDC_ON
	// TODO init SecureDigital and open "MyMap.txt" filename for writing
	myFile = SD.open("MyMap.txt", FILE_WRITE);
#endif

#if GPS_ON
// configures receiving GPS GPRMC message
	gps.begin(9600);
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif		

	// TODO: set target button pinmode
	pinMode(BUT_PIN, INPUT_PULLUP);
}

void loop(void)
{
	float latDeg = 0;
	float lonDeg = 0;

	// for potentiometer
	static float pot_in = 0;
	static int16_t pot_out = 0;
	static uint8_t gammaOutput = 0;

	// get GPS message
	float relativeBearing = 0;
	char* cstr = getGpsMessage();

	// if valid message delivered (happens once a second)
	if (cstr)
	{
#if LOG_ON
		// print the GPRMC message
		Serial.println(cstr);
#endif

		// TODO check button for incrementing target index 0..3
		if (digitalRead(BUT_PIN) == LOW)
		{
			target++;
			if (target == 3)
			{
				target = 0;
			}
		}
		Serial.print("Current target: ");
		Serial.println(target);

		// TODO parse 5 parameters latitude, longitude, and hemisphere indicators, and course over ground from GPS message
		strtok(cstr, ",");
		strtok(NULL, ",");
		strtok(NULL, ",");
		char* lat = strtok(NULL, ",");
		char* latIn = strtok(NULL, ",");
		char* lon = strtok(NULL, ",");
		char* lonIn = strtok(NULL, ",");
		strtok(NULL, ",");
		char* cog = strtok(NULL, ",");
		//"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C"

		// TODO call degMin2DecDeg() convert latitude deg/min to dec/deg
		latDeg = degMin2DecDeg(latIn, lat);

		// TODO call degMin2DecDeg() convert longitude deg/min to dec/deg
		lonDeg = degMin2DecDeg(lonIn, lon);

		// TODO call calcdistance() calculate distance to target
		distance = calcDistance(latDeg, lonDeg, GEOLAT0, GEOLON0);

		// TODO call calcBearing() calculate bearing to target
		heading = calcBearing(latDeg, lonDeg, GEOLAT0, GEOLON0);

		// TODO calculate relative bearing within range >= 0 and < 360
		relativeBearing = heading - strtod(cog, NULL);
		if (relativeBearing >= 360)
		{
			relativeBearing = relativeBearing - 360;
		}
		else if (relativeBearing < 0)
		{
			relativeBearing = relativeBearing + 360;
		}
		else
		{

		}

#if LOG_ON
		Serial.print("Relative Bearing: ");
		Serial.println(relativeBearing);
#endif

#if SDC_ON
		// TODO write required data to SecureDigital then execute flush()

		if (myFile) {
			Serial.print("Writing to MyMap.txt...");
			myFile.println("testing 1, 2, 3.");
			// close the file:
			//myFile.close();
			flush();
			Serial.println("done.");
	}
		else {
			// if the file didn't open, print an error:
			Serial.println("error opening test.txt");
		}
#endif

#if NEO_ON
		// TODO call setNeoPixel() to display target, distance & direction on NeoPixel
		pot_in = analogRead(POT_PIN);
		pot_out = map(pot_in, 0, 1023, 0, 255);
		gammaOutput = pgm_read_byte(&gamma[output]);

		setNeoPixel(target, heading, distance, gammaOutput);
#endif			
	}
}