#ifndef STORMPCI_H
#define STORMPCI_H

#include <linux/types.h>

// The Public API

#define BOLTEK_BUFFERSIZE 512   // Capture Buffer goes from 0 - 511 = 512 bytes.

// tSATELLITETYPE is the detailed satellite info from the gps
//
struct StormProcess_tSATELLITETYPE
{
        char SVID; // 0..37
        char mode; // 0..8
        // where 0 = code search   5 = message sync detect
        //       1 = code acquire  6 = satellite time available
        //       2 = AGC set       7 = ephemeris acquire
        //       3 = freq acquire  8 available for position
        //       4 = bit sync detect
        unsigned char signal_strength; // 0..255
        unsigned short channel_status; // 16 bits
        // where (msb) bit 15 = reserved
        //             bit 14 = reserved
        //             bit 13 = reserved
        //             bit 12 = narrow-band search mode
        //             bit 11 = channel used for time solution
        //             bit 10 = differential corrections available
        //             bit  9 = invalid data
        //             bit  8 = parity error
        //             bit  7 = channel used for position fix
        //             bit  6 = satellite momentum alert flag
        //             bit  5 = satellite anti-spoof flag set
        //             bit  4 = satellite reported unhealthy
        //             bit  3 = satellite accuracy (msb)
        //             bit  2 = satellite accuracy
        //             bit  1 = satellite accuracy 
        //             bit  0 = satellite accuracy (lsb)
};


// tTIMESTAMPINFO is the detailed gps and timestamp data
//
typedef struct 
{
        int TS_valid;
        unsigned long TS_Osc; // actual frequency of timestamp's 50MHz osc
        unsigned long TS_time; // 0..999,999,999 ns of trigger
        unsigned char TS_10ms; // 0..99
        unsigned long capture_time; // 0..999,999,999 ns of first peak

        int gps_data_valid;
        unsigned char month; // 1..12
        unsigned char day; // 1..31
        unsigned short year; // 1998 to 2079
        unsigned char hours; // 0..23
        unsigned char minutes; // 0..59
        unsigned char seconds; // 0..60
        int latitude_mas; // 324,000,000..324,000,000 (-90°..+90°)
        int longitude_mas; // 648,000,000..648,000,000 (-180°..+176°)
        char latitude_ns; // 'N' or 'S'
        char longitude_ew; // 'E' or 'W'
        int height_cm; // -100,000..+1,800,000 (-1000..+18,000m)
        unsigned short dop; // 0..999 (0.0 to 99.9 DOP)
        unsigned char satellites_visible; // 0..12
        unsigned char satellites_tracked; // 0..12
        struct StormProcess_tSATELLITETYPE satellite[12]; // individual satellite data
        unsigned short receiver_status; // 16 bits
        // where (msb) bit 15..13    111 = 3D fix
        //                           110 = 2D fix
        //                           101 = propagate mode
        //                           100 = position hold
        //                           011 = acquiring satellites
        //                           010 = bad geometry
        //                           001 = reserved
        //                           000 = reserved
        //             bit 12..11 = reserved
        //             bit 10 = narrow band tracking mode
        //             bit  9 = fast acquisition position
        //             bit  8 = filter reset to raw gps solution
        //             bit  7 = cold start (no almanac or almanac out of date or time & position unknown)
        //             bit  6 = differential fix
        //             bit  5 = position lock
        //             bit  4 = autosurvey mode
        //             bit  3 = insufficient visible satellites
        //             bit  2..1 = antenna sense
        //               where 00 = Ok
        //                     01 = over current
        //                     10 = under current
        //                     11 = no bias voltage
        //             bit  0 = code location
        short oscillator_temperature; // -110..250 half degrees C (-55..+125°C)
        short serial_number;
} StormProcess_tTIMESTAMPINFO;


typedef struct StormProcess_tBOARDDATA_t
{
        int EFieldBuf[BOLTEK_BUFFERSIZE]; // essentially a 1 bit a-to-d converter
        int NorthBuf[BOLTEK_BUFFERSIZE]; // 0-255 data, we use int so we have room to filter the data
        int EastBuf[BOLTEK_BUFFERSIZE]; // 0-255 data,  we use int so we have room to filter the data
        
        StormProcess_tTIMESTAMPINFO lts2_data; // timestamp and gps data
        int NorthMaxPos, NorthMinPos, EastMaxPos, EastMinPos; // pos of signal peaks
        int North_Pk, East_Pk;               // signal pk-pk amplitude
        int NorthPol, EastPol, EFieldPol;         // signal polarity
} StormProcess_tBOARDDATA;


// this is the structure passed to the device driver GET_DATA ioctl
typedef struct stormpci_packed_data
{
        __u16 usNorth[BOLTEK_BUFFERSIZE];
        __u16 usWest [BOLTEK_BUFFERSIZE];
} StormProcess_tPACKEDDATA;


typedef struct StormProcess_tSTRIKE {
	int valid; // data appars to be valid signal, not just noise
	float distance;  // miles away, for close strike detection
	float distance_averaged;  // miles away, for close strike detection
	float direction; // 0-360 degrees
} StormProcess_tSTRIKE;


#define STORMTRACKER_DEVICE_NAME "/dev/lightning-0"

// connect to the StormTracker card - non-zero on success
int  StormPCI_OpenPciCard(void); 

// clean up, all done
void StormPCI_ClosePciCard(void); 

// after reading the data, wait for the next strike
void StormPCI_RestartBoard(void); 

// force StormTracker to give you a capture
void StormPCI_ForceTrigger(void); 

// check if a strike is waiting to be read by GetCapture()
int  StormPCI_StrikeReady(void); 

// 0-15, 0:most sensitive (preferred), 15: least sensitive
void StormPCI_SetSquelch(char trig_level);

// retrieve the waiting capture
void StormPCI_GetBoardData(StormProcess_tPACKEDDATA* board_data);

void StormProcess_UnpackCaptureData(StormProcess_tPACKEDDATA *packed_data, StormProcess_tBOARDDATA* board_data);

StormProcess_tSTRIKE StormProcess_SSProcessCapture(StormProcess_tBOARDDATA* capture);



#endif

