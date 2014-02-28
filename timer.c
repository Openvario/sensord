/*  
	sensord - Sensor Interface for XCSoar Glide Computer - http://www.openvario.org/
    Copyright (C) 2014  The openvario project
    A detailed list of copyright holders can be found in the file "AUTHORS" 

    This program is free software; you can redistribute it and/or 
    modify it under the terms of the GNU General Public License 
    as published by the Free Software Foundation; either version 3
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, see <http://www.gnu.org/licenses/>.	
*/


/*
//Set the timer  
timer_t SetTimer(int signo, int sec, int mode)
{
	struct sigevent sigev;
  timer_t timerid;
  struct itimerspec itval;
  struct itimerspec oitval;
  
	// Create the POSIX timer to generate signo
  sigev.sigev_notify = SIGEV_SIGNAL;
  sigev.sigev_signo = signo;
  sigev.sigev_value.sival_ptr = &timerid;

  //Create the Timer using timer_create signal

  if (timer_create(CLOCK_REALTIME, &sigev, &timerid) == 0)
  {
		itval.it_value.tv_sec = sec / 1000;
    itval.it_value.tv_nsec = (long)(sec % 1000) * (1000000L);

    if (mode == 1)
    {
			itval.it_interval.tv_sec    = itval.it_value.tv_sec;
      itval.it_interval.tv_nsec = itval.it_value.tv_nsec;
    }
    else
		{
			itval.it_interval.tv_sec = 0;
      itval.it_interval.tv_nsec = 0;
		}
    
		//Set the Timer when to expire through timer_settime
    if (timer_settime(timerid, 0, &itval, &oitval) != 0)
		{
			perror("time_settime error!");
    }
	}

  else
	{
		perror("timer_create error!");
    return -1;
	}
  return timerid;
}
*/

/*
void SignalHandler(int signo, siginfo_t * info, void *context)
{
	unsigned char buf[50];
	
	//printf("wake\n");
	
	if (signo == MEASTIMER)
	{
		switch(state)
		{
			case sample_d1:
				// start conversion
				buf[0] = 0x48;													// This is the register we want to read from
				if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
					printf("Error writing to i2c slave\n");
					exit(1);
				}
				
				state = get_d1;
				break;
				
			case get_d1:

				// read result
				buf[0] = 0x00;
				if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
					printf("Error writing to i2c slave\n");
					exit(1);
				}
		
				if (read(fd, buf, 3) != 3) {								// Read back data into buf[]
					printf("Unable to read from slave\n");
					exit(1);
				}
			
				static_sensor.D1 = buf[0] * 65536 + buf[1] * 256 + buf[2];
				printf("D1 = %lu\n", static_sensor.D1);
				state = sample_d2;
				break;
			
			case sample_d2:
				// start conversion
				buf[0] = 0x58;													// This is the register we want to read from
				if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
					printf("Error writing to i2c slave\n");
					exit(1);
				}
				
				state = get_d2;
				break;
				
			case get_d2:

				// read result
				buf[0] = 0x00;
				if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
					printf("Error writing to i2c slave\n");
					exit(1);
				}
		
				if (read(fd, buf, 3) != 3) {								// Read back data into buf[]
					printf("Unable to read from slave\n");
					exit(1);
				}
				
				static_sensor.D2 = buf[0] * 65536 + buf[1] * 256 + buf[2];
				printf("D2 = %lu\n", static_sensor.D2);
				state = sample_d1;
				break;
				
			default:
				break;
		}
		
  }
}

*/