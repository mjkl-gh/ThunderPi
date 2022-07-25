/* The userspace library for the Boltek Lightning Detector SDK
   Presents a consistent API and .so to userspace, and calls the device
   driver functions
*/

// rev 1.0 072708

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "stormpci.h"

#define BOLTEK_IOCTL_RESTART		_IO  (0xEA, 0xA0)
#define BOLTEK_IOCTL_FORCE_TRIGGER	_IO  (0xEA, 0xA1)
#define BOLTEK_IOCTL_STRIKE_READY	_IOR (0xEA, 0xA2, __u8)
#define BOLTEK_IOCTL_GET_DATA	        _IOR (0xEA, 0xA3, struct stormpci_packed_data)
#define BOLTEK_IOCTL_SET_SQUELCH        _IOW (0xEA, 0xA4, __u8)

static int fd = -1;

typedef int bool;
#define false 0
#define true 1


// connect to the StormTracker card - non-zero on success
int StormPCI_OpenPciCard()
{
        int lfd;
        
        if (fd != -1) return 0;
        
        lfd = open (STORMTRACKER_DEVICE_NAME, O_RDONLY);
        
        if (lfd != -1)
        {
                fd = lfd;
                return 1;
        }
        return 0;
}


// clean up, all done
void StormPCI_ClosePciCard()
{
        close (fd);
        fd = -1;
        return;
}

// after reading the data, wait for the next strike
void StormPCI_RestartBoard()
{
        if (fd == -1) return;
        ioctl (fd, BOLTEK_IOCTL_RESTART);
        return;
}

// force StormTracker to give you a capture
void StormPCI_ForceTrigger()
{
        if (fd == -1) return;
        ioctl (fd, BOLTEK_IOCTL_FORCE_TRIGGER);
        return;
}

// check if a strike is waiting to be read by GetCapture()
// non-zero if a strike is ready
int  StormPCI_StrikeReady()
{
        __u8 datachar;
        
        if (fd == -1) return 0;
        ioctl (fd, BOLTEK_IOCTL_STRIKE_READY, &datachar);
        return (int) datachar;
}

// 0-15, 0:most sensitive (preferred), 15: least sensitive
void StormPCI_SetSquelch(char trig_level)
{
        __u8 datachar;

        if (fd == -1) return;
        datachar = trig_level;
        ioctl (fd, BOLTEK_IOCTL_SET_SQUELCH, &datachar);
        return;
}


//==================================================================
static StormProcess_tTIMESTAMPINFO 
ExtractGPSData(StormProcess_tPACKEDDATA *packeddata)
{
	StormProcess_tTIMESTAMPINFO tgps;
	unsigned char gpsdata[200];		// gps sentence
	unsigned char checksum = 0;
	int cnt;

	// Put timestamp data into gpsdata[] temporarily
	for (cnt=0; cnt < 10; cnt++)
	{
		gpsdata[cnt] = ((packeddata->usWest[cnt] & 0xff00) >> 8);	// 9 bytes of timestamp data before gps data
	}
	
	// Is the TS data valid?
	for (cnt = 0; cnt <9; cnt++)
	{	checksum += gpsdata[cnt];	}
	tgps.TS_valid = ((checksum&0xff) == gpsdata[8]) ? true : false;

	tgps.TS_10ms = gpsdata[0];
	tgps.TS_time = gpsdata[1] + gpsdata[2]*256 + gpsdata[3]*65536 + gpsdata[4]*16777216;
	tgps.TS_Osc  = gpsdata[5] + gpsdata[6]*256 + gpsdata[7]*65536 + gpsdata[8]*16777216;

	// extract gps data from capture
	for (cnt=0; cnt < 157; cnt++)
	{
		gpsdata[cnt] = ((packeddata->usWest[cnt+10] & 0xff00) >> 8);	// 9 bytes of timestamp data before gps data
	}

	// Is the GPS data valid?
	checksum = 0;
	for (cnt = 2; cnt <150; cnt++)
	{	checksum ^= gpsdata[cnt];	}
	tgps.gps_data_valid = (checksum == gpsdata[151]) ? true : false;

	// Date/Time
	tgps.month = gpsdata[4]; 
	tgps.day = gpsdata[5];
	tgps.year = gpsdata[6] * 256 + gpsdata[7];
	tgps.hours = gpsdata[8];
	tgps.minutes = gpsdata[9];
	tgps.seconds = gpsdata[10];

	// Latitude/Longitude
	tgps.latitude_mas = gpsdata[18] + (gpsdata[17]*256) + (gpsdata[16]*65536) + (gpsdata[15]*16777216);
	tgps.longitude_mas = gpsdata[22] + (gpsdata[21]*256) + (gpsdata[20]*65536) + (gpsdata[19]*16777216);
	tgps.height_cm = gpsdata[26] + (gpsdata[25]*256) + (gpsdata[24]*65536) + (gpsdata[23]*16777216);

	if (tgps.latitude_mas<0) {
	   tgps.latitude_mas=abs(tgps.latitude_mas);
	   tgps.latitude_ns='S';
	}
	else
		tgps.latitude_ns='N';
        
	if (tgps.longitude_mas<0) {
                tgps.longitude_mas=abs(tgps.longitude_mas);
                tgps.longitude_ew='W';
	}
	else
		tgps.longitude_ew='E';
        
	// Satellite Status
	tgps.dop = gpsdata[54] + (gpsdata[53] * 256);
	tgps.satellites_visible = gpsdata[55];
	tgps.satellites_tracked = gpsdata[56];
        
	for (cnt=0; cnt < 12; cnt++)
	{
		tgps.satellite[cnt].SVID = gpsdata[57+(cnt*6)];
		tgps.satellite[cnt].mode = gpsdata[58+(cnt*6)];
		tgps.satellite[cnt].signal_strength = gpsdata[59+(cnt*6)];
		tgps.satellite[cnt].channel_status = gpsdata[62+(cnt*6)] + (gpsdata[61+(cnt*6)]*256);
	}
        
	tgps.receiver_status = gpsdata[130] + (gpsdata[129]*256);
	tgps.oscillator_temperature = (gpsdata[140] + (gpsdata[139]*256))>>1; // convert half degrees to degrees C
	tgps.serial_number = gpsdata[156] + (gpsdata[155]*256);
        
	return tgps;
}




#define MAXBUFVAL 1020    /*  1020 = 255 * 4 byte filter  */
#define CLIPEXTRAPVAL 10	/*  how much bigger should the signal be, if we clipped  */
#define E_FIELD_OFFSET 10    /*  E-Field leads H-Field  */
#define FREQUENCYCHECK 45    /*  Min and Max must be this far apart to be Valid  */
#define SUCKIN 0.29 // % of screen we suck inwards from center, so strikes happen near us
#define SUCKSUBTRACT R_SCREEN_LIMIT * SUCKIN    /*  subtract this const from strike distance  */
/*  Mult strike dist by this after subtract to put far strikes back at edge  */
#define SCREENSCALINGCONSTANT 23.0    /*  adjusts how far out from center strikes fall */
#define R_SCREEN_LIMIT 0.09235    /*  0.121; Capture.Process limits real X/Y to this  */
/*  Process(): Miles = StrikeDistance*MilesConstant, for Close Storm Alarm  */
#define MILESCONSTANT (R_SCREEN_LIMIT/300)    /*  decrease 3?? to decrease miles  */
#define SUCKMULTIPLY R_SCREEN_LIMIT / (R_SCREEN_LIMIT - SUCKSUBTRACT)
#define AVERAGEDURATION 400    /*  replace average strike dist after this many demis  */
#define DELTAPLUSLIMITMVAL 55    /*  Limit far strikes pull out = (m*strikerate) + b */
#define DELTAMINUSFILTERVAL 0.68 // Reduce impact of close in strikes 0-1.0x
#define DELTAPLUSLIMITBVAL 400    /*  Limit far strikes pull out when low strikerate  */



void
StormProcess_UnpackCaptureData(StormProcess_tPACKEDDATA *packed_data, StormProcess_tBOARDDATA* board_data)
{
        int cnt;
        
        for (cnt = 0; cnt < BOLTEK_BUFFERSIZE; cnt++)
        {
                board_data->NorthBuf[cnt] = packed_data->usNorth[cnt] & 0xff;
                board_data->EastBuf[cnt] = packed_data->usWest[cnt] & 0xff;
                board_data->EFieldBuf[cnt] = (packed_data->usNorth[cnt] & 0x0100) == 0x100;
        }
        // Fill the tTIMESTAMPINFO lts2_data fields
        board_data->lts2_data = ExtractGPSData(packed_data);
        return;
}

static void 
Capture_Filter(StormProcess_tBOARDDATA* capture)
{
        int      X;
        
        for (X = 0; X < BOLTEK_BUFFERSIZE - 4; X++) 
        {
                capture->NorthBuf[X] = (capture->NorthBuf[X] + capture->NorthBuf[X + 1] +
                                        capture->NorthBuf[X + 2] + capture->NorthBuf[X + 3]);   /* shr 2;   shr puts us back 0-255 temp  */
                capture->EastBuf[X] = (capture->EastBuf[X] + capture->EastBuf[X + 1] + capture->EastBuf[X + 2] +
                                       capture->EastBuf[X + 3]);   /* shr 2;  shr puts us back 0-255 temp  */
        } 
        /*  Now take care of the last few  */ 
        capture->NorthBuf[BOLTEK_BUFFERSIZE - 4] = capture->NorthBuf[BOLTEK_BUFFERSIZE - 4] + capture->NorthBuf[BOLTEK_BUFFERSIZE - 3] +
                capture->NorthBuf[BOLTEK_BUFFERSIZE - 2] + capture->NorthBuf[BOLTEK_BUFFERSIZE - 1];
        capture->NorthBuf[BOLTEK_BUFFERSIZE - 3] = capture->NorthBuf[BOLTEK_BUFFERSIZE - 4];
        capture->NorthBuf[BOLTEK_BUFFERSIZE - 2] = capture->NorthBuf[BOLTEK_BUFFERSIZE - 4];
        capture->NorthBuf[BOLTEK_BUFFERSIZE - 1] = capture->NorthBuf[BOLTEK_BUFFERSIZE - 4];
        capture->EastBuf[BOLTEK_BUFFERSIZE - 4] = capture->EastBuf[BOLTEK_BUFFERSIZE - 4] + capture->EastBuf[BOLTEK_BUFFERSIZE - 3] + 
                capture->EastBuf[BOLTEK_BUFFERSIZE - 2] + capture->EastBuf[BOLTEK_BUFFERSIZE - 1];
        capture->EastBuf[BOLTEK_BUFFERSIZE - 3] = capture->EastBuf[BOLTEK_BUFFERSIZE - 4];
        capture->EastBuf[BOLTEK_BUFFERSIZE - 2] = capture->EastBuf[BOLTEK_BUFFERSIZE - 4];
        capture->EastBuf[BOLTEK_BUFFERSIZE - 1] = capture->EastBuf[BOLTEK_BUFFERSIZE - 4]; 
        return;
}

static void 
Capture_Find_Peaks(StormProcess_tBOARDDATA* capture)
/*
  Original Find Peaks algorithm. No clipping here
*/
{
        int      NorthClipValue, EastClipValue, Count, NorthClipPositive, NorthClipNegative,
                EastClipPositive, EastClipNegative, NorthPkMax, NorthPkMin, EastPkMax, EastPkMin;
        
        NorthPkMax = 0;
        EastPkMax = 0;
        NorthPkMin = 32000;
        EastPkMin = 32000;
        NorthClipValue = 0; 
        EastClipValue = 0;
        NorthClipPositive = 0;   /*  zero clip counter  */
        NorthClipNegative = 0;   /*  zero clip counter  */
        EastClipPositive = 0;   /*  zero clip counter  */
        EastClipNegative = 0;   /*  zero clip counter  */ 
        /*  Don't start at position zero since we often have bad sample(s?) there  */ 
        for (Count = 3; Count < BOLTEK_BUFFERSIZE; Count++) {
                if (capture->NorthBuf[Count] > NorthPkMax) { 
                        NorthPkMax = capture->NorthBuf[Count];
                        capture->NorthMaxPos = Count;
                } 
                if (capture->NorthBuf[Count] < NorthPkMin) { 
                        NorthPkMin = capture->NorthBuf[Count];
                        capture->NorthMinPos = Count;
                } 
                if (capture->EastBuf[Count] > EastPkMax) {
                        EastPkMax = capture->EastBuf[Count];
                        capture->EastMaxPos = Count;
                }
                if (capture->EastBuf[Count] < EastPkMin) {
                        EastPkMin = capture->EastBuf[Count];
                        capture->EastMinPos = Count; 
                } 
		/*  NOW CHECK FOR CLIPPING SIGNAL: IF WE ARE STUCK AT MaxBufVal THEN  */
		/*  WE ARE CLIPPING SO EXTRAPOLATE UPWARDS BASED ON A STRAIGHT LINE  */
                /*  CONSTANT: ClipExtrapVal IS THE SLOPE OF THE LINE  */ 
                
                /*  Check for North Clipping +'ve  */ 
                if (capture->NorthBuf[Count] == MAXBUFVAL) {
                        NorthClipValue = NorthClipValue + CLIPEXTRAPVAL;
                        NorthClipPositive++;   /*  counter for peak location  */
                }
		/*  Check for North Clipping -'ve  */
                if (capture->NorthBuf[Count] == 0) {
                        NorthClipValue = NorthClipValue + CLIPEXTRAPVAL;
                        NorthClipNegative++;   /*  counter for peak location  */
                }
		/*  Check for East Clipping +'ve  */
                if (capture->EastBuf[Count] == MAXBUFVAL) {
                        EastClipValue = EastClipValue + CLIPEXTRAPVAL;
                        EastClipPositive++;   /*  counter for peak location  */
                }
		/*  Check for East Clipping -'ve  */
                if (capture->EastBuf[Count] == 0) {
                        EastClipValue = EastClipValue + CLIPEXTRAPVAL;
                        EastClipNegative++;   /*  counter for peak location  */
                }
        }
        /*  If we clipped then move the increase the peak location so that
            peak is the middle of the clipped off peak, not the start  */
        capture->NorthMaxPos = capture->NorthMaxPos + (NorthClipPositive / 2);
        capture->NorthMinPos = capture->NorthMinPos + (NorthClipNegative / 2);
        capture->EastMaxPos = capture->EastMaxPos + (EastClipPositive / 2);
        capture->EastMinPos = capture->EastMinPos + (EastClipNegative / 2);
        
        capture->North_Pk = NorthPkMax - NorthPkMin;
        capture->East_Pk = EastPkMax - EastPkMin; 
        
        /*  Now, the largest signal determines where we measure signals  */
        if (capture->North_Pk > capture->East_Pk) { 
		/*  NORTH WINS  */
                capture->EastMinPos = capture->NorthMinPos; 
                capture->EastMaxPos = capture->NorthMaxPos; 
                
		/* Remeasure East signal with new positions  */
                capture->East_Pk = capture->EastBuf[capture->EastMaxPos] - capture->EastBuf[capture->EastMinPos];
                if (capture->East_Pk < 0) { 
                        /*  We've got min and max reversed  */ 
                        capture->EastMinPos = capture->NorthMaxPos;
                        capture->EastMaxPos = capture->NorthMinPos;
                        capture->East_Pk = -capture->East_Pk;
                } 
        }
        else {   /*  EAST WINS  */
                capture->NorthMinPos = capture->EastMinPos; 
                capture->NorthMaxPos = capture->EastMaxPos;
                
                /* Remeasure North signal with new positions  */
                capture->North_Pk = capture->NorthBuf[capture->NorthMaxPos] - capture->NorthBuf[capture->NorthMinPos];
                if (capture->North_Pk < 0) { 
                        /*  We've got min and max reversed  */ 
                        capture->NorthMinPos = capture->EastMaxPos;
                        capture->NorthMaxPos = capture->EastMinPos;
                        capture->North_Pk = -capture->North_Pk;
		}
        }
        /*  Add any extra from clipping  */
        capture->North_Pk = capture->North_Pk + NorthClipValue;
        capture->East_Pk = capture->East_Pk + EastClipValue;
        return;
}


static bool 
Capture_Valid(StormProcess_tBOARDDATA* capture)
/*
  This fuunction must execute to the end since E_Field_Polarity
  is figured out here.
*/
{
        bool CapValid = true;
        int NorthMinE_FCheck, NorthMaxE_FCheck, EastMinE_FCheck, EastMaxE_FCheck;
        /*
          Check E-Field Coherency
          Invalid if E-Field is same polarity at min & max.
          E_Field is not exactly in phase with H field, so Offset E Field position
	*/
        if (capture->NorthMinPos > E_FIELD_OFFSET)
                NorthMinE_FCheck = capture->EFieldBuf[capture->NorthMinPos - E_FIELD_OFFSET];
        else
                NorthMinE_FCheck = capture->EFieldBuf[capture->NorthMinPos];

        if (capture->NorthMaxPos > E_FIELD_OFFSET)
                NorthMaxE_FCheck = capture->EFieldBuf[capture->NorthMaxPos - E_FIELD_OFFSET];
        else
                NorthMaxE_FCheck = capture->EFieldBuf[capture->NorthMaxPos];

        if (capture->EastMinPos > E_FIELD_OFFSET)
                EastMinE_FCheck = capture->EFieldBuf[capture->EastMinPos - E_FIELD_OFFSET];
        else
                EastMinE_FCheck = capture->EFieldBuf[capture->EastMinPos];

        if (capture->EastMaxPos > E_FIELD_OFFSET)
                EastMaxE_FCheck = capture->EFieldBuf[capture->EastMaxPos - E_FIELD_OFFSET];
        else
                EastMaxE_FCheck = capture->EFieldBuf[capture->EastMaxPos];

        /*  If E Field doesn't change during capture then E Field not valid  */
        if ((NorthMinE_FCheck == NorthMaxE_FCheck) && (EastMinE_FCheck == EastMaxE_FCheck))
		CapValid = false;

        /*  IF MIN AND MAX ARE TOO CLOSE THEN THIS IS HIGH FREQ NOISE  */
        if (abs(capture->NorthMaxPos - capture->NorthMinPos) < FREQUENCYCHECK) CapValid = false;

        /*
          THIS STUFF DOESN'T CONCERN VALID().
          IT IS JUST A GOOD PLACE TO FIGURE IT OUT.
	*/
        /*  Figure out E_Field Polarity.  */
        /*  Either North or East channel will have the polarity answer.  */

        if ((NorthMinE_FCheck != NorthMaxE_FCheck)) {
                if (capture->NorthMinPos < capture->NorthMaxPos)
                        capture->EFieldPol = NorthMinE_FCheck;
                else
                        capture->EFieldPol = NorthMaxE_FCheck;
        }
        else
                if (capture->EastMinPos < capture->EastMaxPos)
                        capture->EFieldPol = EastMinE_FCheck;
                else
                        capture->EFieldPol = EastMaxE_FCheck;

        /*  Set Polarities: If same polarity as E Field then +1
            If opposite polarity        then -1   */

        if ((capture->NorthMaxPos < capture->NorthMinPos) == capture->EFieldPol)
                capture->NorthPol = 1;
        else
                capture->NorthPol = -1;

        /*  East polarity is opposite North due to antenna phase connections  */
        if ((capture->EastMaxPos < capture->EastMinPos) == capture->EFieldPol)
                capture->EastPol = 1;
        else
                capture->EastPol = -1;

        return CapValid;
}


static time_t 
CurrentTime() {
	time_t now;
	time(&now);
	return now;
}

static StormProcess_tSTRIKE 
Capture_ConvertToStrike(StormProcess_tBOARDDATA* capture)
/*
  turns raw capture data into strike data
  Generates integer X and Y values for the logfiles.
  The plot routine will scale this for the type of display.

  y offset from center = k N / ( N*N + E*E )
  x offset from center = k E / ( N*N + E*E )


  AVERAGING ALGORITHM
  Program.Init calls ZeroAverages
  Capture.Process adjusts the distance of the strike according to
  average[bearing]. We average each new distance into the current
  average distance to correct for weak strikes received in close
  storms.
  Delta is the amount the new strike moves the average.
  +'ve Delta means the new strike was farther from the current average.
  -'ve Delta means the new strike was closer than the current average.
  To control pie-slicing we limit +'ve Delta. A close storm will have
  a large number of far strikes. The impact of these strikes must be
  reduced so they do not move our average farther than it should be.
  -ve Delta is what brings the storm closer to us. That is controlled
  only by the averaging.
  Later- CheckColorUnder sets Last flag in strike record if it finds
  a strike at the same direction.
  Later- Unplot sets average back to zero when it sees Last flag set.

  SUCK-IN ALGORITHM
  We never get strikes happening near us. The storm jumps over the center
  portion of the screen. To correct for this we suck-in the center of the
  screen, with the edge of the screen staying where it is. This stretches
  the rest of the screen.
  To suck-in: subtract SuckInSubtract from each strikes distance.
  To stretch far strikes back to the edge: mult the result by SuckMultiply
*/
{
        static double Average[366]; // average distance[365 degrees+1]
        static time_t AverageTime[366];  // time when average was set[365 degrees+1]
  
        double R_XValue, R_YValue, Divisor;
        double Delta, Delta2, Delta3, Delta4, Delta5, Delta6, Delta7,
                MaxDelta, Distance, New_Distance, North_Pk_Real, East_Pk_Real;
        double d_bearing;
        int bearing, adjacentbearing, I_XValue, I_YValue;
        StormProcess_tSTRIKE new_strike;

        North_Pk_Real = (float)capture->North_Pk;   /*  must convert to real, since we overload integer  */
        East_Pk_Real = (float)capture->East_Pk;

        Divisor = North_Pk_Real * North_Pk_Real + East_Pk_Real * East_Pk_Real;

        /*  Raise divisor to the exponent 3/4  */
        Divisor = (float)sqrt(sqrt(Divisor * Divisor * Divisor));

        /*  check for divide by zero  */
        if (Divisor != 0) {
                R_XValue = capture->EastPol * East_Pk_Real / Divisor;
                R_YValue = capture->NorthPol * North_Pk_Real / Divisor;
        }
        else {
                R_XValue = (float)capture->EastPol;   /*  creates an integer of 32256  */
                R_YValue = (float)capture->NorthPol;   /*  creates an integer of 32256  */
        }

        /*  CALCULATE STRIKE POSITION  */

        /*  AVERAGING - To eliminate pie slicing  */
        /*  Averaging works just like averaging on a waveform. We only let each
            new sample adjust the average value by a certain percentage. For example
            only 20% of change is signal is allowed. This limits the high freq., or
            in our case prevents radical strikes. One close in and the next one
            at the edge of the screen. We divide the screen into 360 one degree
            sections. Each section is allowed only one average distance. Each
            strike can adjust that average distance only slightly.
            { Calc strike bearing and distance  */
        /*  strike degrees = 57.3 degrees/rad * strike radians  */
        if (R_XValue != 0)
                d_bearing = (180 + 57.296 * atan(R_YValue / R_XValue));
        else
		/*  X is zero. Look at y to see whether we are up or down.  */
                if (R_YValue > 0)
                        d_bearing = 270.0;
                else
                        d_bearing = 90.0;
        /*  range checking for safety  */
        if ((d_bearing > 359.0) || (d_bearing < 0.0))
                d_bearing = 359.0;
        bearing = (int) d_bearing;
        Distance = (float)sqrt((R_XValue * R_XValue) + (R_YValue * R_YValue));

        /*  NOW SUCK IN THE CENTER OF THE SCREEN  */
        New_Distance = (float)(Distance - SUCKSUBTRACT);
        /*  check if we sucked past the center  */
        if (New_Distance < 0) New_Distance = 0.0;
        /*  stretch far strikes back to the edge  */
        New_Distance = (float)(New_Distance * SUCKMULTIPLY);
        /*  Calc the new real X and Y values for this sucked in location  */
        if (Distance != 0) {
                R_XValue = R_XValue * New_Distance / Distance;
                R_YValue = R_YValue * New_Distance / Distance;
        }
        else {
		/*  Distance is zero, so New_Distance is zero, so strike is at zero  */
                R_XValue = 0.0;
                R_YValue = 0.0;
        }

        /*  BACK TO THE AVERAGING  */
        /*  Check if average = zero. Set average to this strike  */
        if ( (CurrentTime()-AverageTime[bearing]) > AVERAGEDURATION) {
		/*  AVERAGE HASN'T BEEN SET YET  */
                Average[bearing] = New_Distance;
                AverageTime[bearing] = CurrentTime();   /*  timestamp the average  */
        }
        else {
                /*  FACTOR IN NEW STRIKE  */
                /*  Calc how much this strike changes our current average  */
                Delta  = New_Distance - Average[bearing];
                Delta2 = New_Distance - Average[bearing - 1];   /*  adjacent degree  */
                Delta3 = New_Distance - Average[bearing + 1];   /*  adjacent degree  */
                Delta4 = New_Distance - Average[bearing - 2];   /*  adjacent degree  */
                Delta5 = New_Distance - Average[bearing + 2];   /*  adjacent degree  */
                Delta6 = New_Distance - Average[bearing - 3];    /*  adjacent degree  */
                Delta7 = New_Distance - Average[bearing + 3];    /*  adjacent degree  */
                /* testdelta := Delta; { test  */
                /*  Limit how much one strike can pull us away from center  */
                /*  Limit based on strike rate  */
                MaxDelta = (float)(R_SCREEN_LIMIT / (DELTAPLUSLIMITMVAL *
                                                     /*strike/min + */  DELTAPLUSLIMITBVAL));


                /*  LIMIT FAR STRIKES AND AVERAGE CLOSE STRIKES  */
                if (Delta > MaxDelta)   /*  this bearing's average  */
                        Delta = MaxDelta;
                else
                        Delta = (float)(Delta * DELTAMINUSFILTERVAL);
                if (Delta2 > MaxDelta)   /*  adjacent bearing  */
                        Delta2 = MaxDelta;
                else
                        Delta2 = (float)(Delta2 * DELTAMINUSFILTERVAL);
                if (Delta3 > MaxDelta)   /*  adjacent bearing  */
                        Delta3 = MaxDelta;
                else
                        Delta3 = (float)(Delta3 * DELTAMINUSFILTERVAL);
                if (Delta4 > MaxDelta)   /*  adjacent bearing  */
                        Delta4 = MaxDelta;
                else
                        Delta4 = (float)(Delta4 * DELTAMINUSFILTERVAL);
                if (Delta5 > MaxDelta)   /*  adjacent bearing  */
                        Delta5 = MaxDelta;
                else
                        Delta5 = (float)(Delta5 * DELTAMINUSFILTERVAL);
                if (Delta6 > MaxDelta)               /* adjacent bearing  */
			Delta6 = MaxDelta;
                else
                        Delta6 = (float)(Delta6 * DELTAMINUSFILTERVAL);
                if (Delta7 > MaxDelta)                    /* adjacent bearing  */
			Delta7 = MaxDelta;
                else
                        Delta7 = (float)(Delta7 * DELTAMINUSFILTERVAL);

                /*  Now Adjust average distance for this new strike  */
                // 8/24/98 - Elapsed() > AVERAGEDURATION changed to < AVERAGEDURATION
                Average[bearing] = Average[bearing] + Delta;
                time_t now = CurrentTime();   /*  in demis, to timestamp averages  */
                AverageTime[bearing] = now;   /*  timestamp the average  */

                adjacentbearing = bearing - 1; // smooth distances with adjacent bearing
                if (adjacentbearing < 0 ) adjacentbearing += 360; // wrap around at 0 degrees
                if ( (now-AverageTime[adjacentbearing]) < AVERAGEDURATION)
                        Average[adjacentbearing] = Average[adjacentbearing] + Delta2/2;
                else
                        Average[adjacentbearing] = Average[bearing];   /*  set adjacent to current  */
                AverageTime[adjacentbearing] = now;   /*  timestamp the average  */

                adjacentbearing = bearing + 1; // smooth distances with adjacent bearing
                if (adjacentbearing > 360 ) adjacentbearing -= 360; // wrap around at 0 degrees
                if ( (now-AverageTime[adjacentbearing]) < AVERAGEDURATION)
                        Average[adjacentbearing] = Average[adjacentbearing] + Delta3/2;
                else
                        Average[adjacentbearing] = Average[bearing];   /*  set adjacent to current  */
                AverageTime[adjacentbearing] = now;   /*  timestamp the average  */

                adjacentbearing = bearing - 2; // smooth distances with adjacent bearing
                if (adjacentbearing < 0 ) adjacentbearing += 360; // wrap around at 0 degrees
                if ( (now-AverageTime[adjacentbearing]) < AVERAGEDURATION)
                        Average[adjacentbearing] = Average[adjacentbearing] + Delta4/2;
                else
                        Average[adjacentbearing] = Average[bearing];   /*  set adjacent to current  */
                AverageTime[adjacentbearing] = now;   /*  timestamp the average  */

                adjacentbearing = bearing + 2; // smooth distances with adjacent bearing
                if (adjacentbearing > 360 ) adjacentbearing -= 360; // wrap around at 0 degrees
                if ( (now-AverageTime[adjacentbearing]) < AVERAGEDURATION)
                        Average[adjacentbearing] = Average[adjacentbearing] + Delta5/2;
                else
                        Average[adjacentbearing] = Average[bearing];   /*  set adjacent to current  */
                AverageTime[adjacentbearing] = now;   /*  timestamp the average  */

                adjacentbearing = bearing - 3; // smooth distances with adjacent bearing
                if (adjacentbearing < 0 ) adjacentbearing += 360; // wrap around at 0 degrees
                if ( (now-AverageTime[adjacentbearing]) < AVERAGEDURATION)
                        Average[adjacentbearing] = Average[adjacentbearing] + Delta6/2;
                else
                        Average[adjacentbearing] = Average[bearing];   /*  set adjacent to current  */
                AverageTime[adjacentbearing] = now;   /*  timestamp the average  */

                adjacentbearing = bearing + 3; // smooth distances with adjacent bearing
                if (adjacentbearing > 360 ) adjacentbearing -= 360; // wrap around at 0 degrees
                if ( (now-AverageTime[adjacentbearing]) < AVERAGEDURATION)
                        Average[adjacentbearing] = Average[adjacentbearing] + Delta7/2;
                else
                        Average[adjacentbearing] = Average[bearing];   /*  set adjacent to current  */
                AverageTime[adjacentbearing] = now;   /*  timestamp the average  */

                /*  Now adjust this strike to the current average  */
                /*  scale X and Y according to the ratio of old Average to New_Averge  */

                if (New_Distance != 0) {
                        R_XValue = R_XValue * Average[bearing] / New_Distance;
                        R_YValue = R_YValue * Average[bearing] / New_Distance;
		}
                else {
                        /*  Distance for this strike is zero,
                            so we can't just scale the current values, must calculate them  */
                        R_XValue = (float)(-cos(bearing / 57.296) * Average[bearing]);   /*  angle in radians  */
                        R_YValue = (float)(-sin(bearing / 57.296) * Average[bearing]);   /*  angle in radians  */
                }
        } /* END FACTOR IN NEW STRIKE */

        /*  Check that number will not overflow/underflow integer  */
        /*  Limit value to 7648. Since we are storing data in file at -+32xVGA  */
        /*  resolution this will limit strikes to near edge of screen  */
        if (R_XValue > R_SCREEN_LIMIT) R_XValue = (float)R_SCREEN_LIMIT;
        if (R_XValue < -R_SCREEN_LIMIT) R_XValue = (float)-R_SCREEN_LIMIT;
        if (R_YValue > R_SCREEN_LIMIT) R_YValue = (float)R_SCREEN_LIMIT;
        if (R_YValue < -R_SCREEN_LIMIT) R_YValue = (float)-R_SCREEN_LIMIT;

        /*  Convert to uint variable type, as required by datafile  */
        /*                  scaling constant * 16xVGA resolution  */
        /* I_XValue := trunc(R_XValue * 3960 * 16); displays 40% too close  */
        /* I_YValue := trunc(R_YValue * 3960 * 16);  */
        /* I_XValue := trunc(R_XValue * 3960.0 * 24.0); { displays 25% to far  */
        /* I_YValue := trunc(R_YValue * 3960.0 * 24.0);   */
        I_XValue = (int)(R_XValue * 3960.0 * SCREENSCALINGCONSTANT);
        I_YValue = (int)(R_YValue * 3960.0 * SCREENSCALINGCONSTANT);

        new_strike.distance = (float)New_Distance; // unaveraged
        new_strike.distance_averaged = (float)(Average[bearing] / MILESCONSTANT); // store distance in miles;
        new_strike.direction = (float)d_bearing;

        return new_strike;
} 


//==================================================================
// Perform a single-site strike position calculation
//
StormProcess_tSTRIKE 
StormProcess_SSProcessCapture(StormProcess_tBOARDDATA* capture)
{
	StormProcess_tSTRIKE strike;
        
	Capture_Filter(capture);
	Capture_Find_Peaks(capture);
	
	strike.valid = Capture_Valid(capture); // looks like a strike?
	
	return Capture_ConvertToStrike(capture);
}


// retrieve the waiting capture
void StormPCI_GetBoardData(StormProcess_tPACKEDDATA * board_data)
{
        /* struct stormpci_packed_data aka StormProcess_tPACKEDDATA */
        if (fd == -1) return;

        ioctl (fd, BOLTEK_IOCTL_GET_DATA, board_data);
        return;
}

