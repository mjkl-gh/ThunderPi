/* An Example application for using the Boltek SDK
   remember to load the device driver first */

/* Rev 1.0 - July 27 2008 */

#include <stdio.h>
#include <unistd.h>
#include <time.h>

#include "stormpci.h"

//#define SIMULATE_HIT 1
#define POLL_INTERVAL_SECONDS  15

int main()
{
        int ready, i, j;
        StormProcess_tPACKEDDATA packed_info;
        StormProcess_tBOARDDATA  unpacked_info;
        StormProcess_tSTRIKE strike;
        time_t now;
        
        if (StormPCI_OpenPciCard())
        {
                // squelch is from 0 to 15, with 0 as most sensitive (and default)
                StormPCI_SetSquelch (0);
                
                while (1)
                {
#if SIMULATE_HIT
                        // to simulate a hit use
                        StormPCI_ForceTrigger(); sleep(1);
#endif                   
                        ready = StormPCI_StrikeReady();
                        if (ready)
                        {
                                time(&now);
                                StormPCI_GetBoardData(&packed_info);
                                StormPCI_RestartBoard();

                                StormProcess_UnpackCaptureData(&packed_info, &unpacked_info);
                                strike = StormProcess_SSProcessCapture(&unpacked_info);
                                
                                if (strike.valid)
                                {
                                        printf ("\n\nStrike Detected at: %s", ctime(&now));
                                        printf ("%3.3f miles away (%3.3f averaged)\n",
                                                strike.distance, strike.distance_averaged);
                                        printf ("The strike was at %4.1f degrees\n",
                                                strike.direction);
                                        printf ("\n");
                                }
                                else
                                        printf ("-");
                                
                        }
                        else
                                printf (".");
                        fflush (stdout);
                        
                        sleep (POLL_INTERVAL_SECONDS);
                }
        }
        else
        {
                printf ("Cannot Access Boltek Lightning Detector\n");
        }
        
        StormPCI_ClosePciCard(); 

        return 0;
}

