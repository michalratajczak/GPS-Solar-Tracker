#include "main.h"
#include "stm32f4xx_hal.h"

#ifndef TRACKER_H_
#define TRACKER_H_

//===============================================================================
//variables and structures
UART_HandleTypeDef huart4;


int MODE = 0;//0 - center | 1 - presentation | 2 - normal
int AZIMUTH = 1500;
int ELEVATION = 1500;

double presentation_mode_counter = 0;

uint8_t receiveUART[500];
uint16_t sizeReceiveUART = 500;

struct gps
{
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    double latitude;
    double longitude;

    uint8_t day;
    uint8_t month;
    uint8_t year;
};

struct gps GPS;

//===============================================================================
//functions

int convert_to_int(const char* txt_begin, const char* txt_end)
{
    // alloc
    char* buff = malloc(txt_end - txt_begin);
    memcpy(buff, txt_begin, txt_end - txt_begin);
    int number = atoi(buff);
    // free
    free(buff);

    return number;
}

void extract_data(char* gps_text, struct gps* GPS)
{
    int div_ctr = 0;
    char* txt = gps_text;

    while (txt < &gps_text[strlen(gps_text)])
    {
        if (*txt == ',')
        {
            txt++;
            div_ctr++;
            if (div_ctr == 1)
            {
                GPS->hour = convert_to_int(txt, txt + 2);
                GPS->minute = convert_to_int(txt + 2, txt + 4);
                GPS->second = convert_to_int(txt + 4, txt + 6);
                txt += 9;
            }

            if (div_ctr == 3)
            {
                GPS->latitude = atof(txt);
            }

            if (div_ctr == 5)
            {
                GPS->longitude = atof(txt);
            }

            if (div_ctr == 9)
            {
                GPS->day = convert_to_int(txt, txt + 2);
                GPS->month = convert_to_int(txt + 2, txt + 4);
                GPS->year = convert_to_int(txt + 4, txt + 6);
                break;
            }
        }
        else
        {
            txt++;
        }
    }
}

int parse_input(char* gps_text, size_t len, struct gps* GPS)
{
    for (size_t x = 0; x < len; x++)
    {
        if (gps_text[x] == '$' && gps_text[x + 3] == 'R' && gps_text[x + 4] == 'M' && gps_text[x + 5] == 'C')
        {
            extract_data(&gps_text[x], GPS);
            if(GPS->hour == 0 || GPS->minute == 0 || GPS->second == 0)
            {
                continue;
            }

            if(GPS->latitude == 0 || GPS->longitude == 0)
            {
                continue;
            }

            if(GPS->day == 0 || GPS->month == 0 || GPS->year == 0)
            {
                continue;
            }

            return 1;
        }
    }

    return 0;
}

double radians(double angle)
{
	return 3.14159265359 / 180 * angle;
}

double degrees(double radians)
{
	return 180 / 3.14159265359 * radians;
}

double solar_elevation_angle(double time_zone, double latitude, double longtitude, double local_time, double year)
{
	double D2 = year;
	double E2 = local_time;
	double F2 = D2 + 2415018.5 + E2 - time_zone / 24;
	double G2 = (F2 - 2451545) / 36525;
	double I2 = fmod(280.46646 + G2 * (36000.76983 + G2 * 0.0003032), 360);
	double J2 = 357.52911 + G2 * (35999.05029 - 0.0001537*G2);
	double K2 = 0.016708634 - G2 * (0.000042037 + 0.0000001267*G2);
	double L2 = sin(radians(J2))*(1.914602 - G2 * (0.004817 + 0.000014*G2)) + sin(radians(2 * J2))*(0.019993 - 0.000101*G2) + sin(radians(3 * J2))*0.000289;
	double M2 = I2 + L2;
	double P2 = M2 - 0.00569 - 0.00478*sin(radians(125.04 - 1934.136*G2));
	double Q2 = 23 + (26 + ((21.448 - G2 * (46.815 + G2 * (0.00059 - G2 * 0.001813)))) / 60) / 60;
	double R2 = Q2 + 0.00256*cos(radians(125.04 - 1934.136*G2));
	double T2 = degrees(asin(sin(radians(R2))*sin(radians(P2))));
	double U2 = tan(radians(R2 / 2))*tan(radians(R2 / 2));
	double V2 = 4 * degrees(U2*sin(2 * radians(I2)) - 2 * K2*sin(radians(J2)) + 4 * K2*U2*sin(radians(J2))*cos(2 * radians(I2)) - 0.5*U2*U2*sin(4 * radians(I2)) - 1.25*K2*K2*sin(2 * radians(J2)));
	double AB2 = fmod(E2 * 1440 + V2 + 4 * longtitude - 60 * time_zone, 1440);
	double AC2 = 0;
	if (AB2 / 4 < 0) AC2 = AB2 / 4 + 180;
	else AC2 = AB2 / 4 - 180;
	double AD2 = degrees(acos(sin(radians(latitude))*sin(radians(T2)) + cos(radians(latitude))*cos(radians(T2))*cos(radians(AC2))));

	return 90 - AD2;
}

double solar_azimuth_angle(double time_zone, double latitude, double longtitude, double local_time, double year)
{
	double D2 = year;
	double E2 = local_time;
	double F2 = D2 + 2415018.5 + E2 - time_zone / 24;
	double G2 = (F2 - 2451545) / 36525;
	double I2 = fmod(280.46646 + G2 * (36000.76983 + G2 * 0.0003032), 360);
	double J2 = 357.52911 + G2 * (35999.05029 - 0.0001537*G2);
	double K2 = 0.016708634 - G2 * (0.000042037 + 0.0000001267*G2);
	double L2 = sin(radians(J2))*(1.914602 - G2 * (0.004817 + 0.000014*G2)) + sin(radians(2 * J2))*(0.019993 - 0.000101*G2) + sin(radians(3 * J2))*0.000289;
	double M2 = I2 + L2;
	double P2 = M2 - 0.00569 - 0.00478*sin(radians(125.04 - 1934.136*G2));
	double Q2 = 23 + (26 + ((21.448 - G2 * (46.815 + G2 * (0.00059 - G2 * 0.001813)))) / 60) / 60;
	double R2 = Q2 + 0.00256*cos(radians(125.04 - 1934.136*G2));
	double T2 = degrees(asin(sin(radians(R2))*sin(radians(P2))));
	double U2 = tan(radians(R2 / 2))*tan(radians(R2 / 2));
	double V2 = 4 * degrees(U2*sin(2 * radians(I2)) - 2 * K2*sin(radians(J2)) + 4 * K2*U2*sin(radians(J2))*cos(2 * radians(I2)) - 0.5*U2*U2*sin(4 * radians(I2)) - 1.25*K2*K2*sin(2 * radians(J2)));

	double AB2 = fmod(E2 * 1440 + V2 + 4 * longtitude - 60 * time_zone, 1440);
	double AC2 = 0;
	if (AB2 / 4 < 0) AC2 = AB2 / 4 + 180;
	else AC2 = AB2 / 4 - 180;
	double AD2 = degrees(acos(sin(radians(latitude))*sin(radians(T2)) + cos(radians(latitude))*cos(radians(T2))*cos(radians(AC2))));
	double AH2;
	if(AC2 > 0)
		return (double)((long)(degrees(acos(((sin(radians(latitude))*cos(radians(AD2))) - sin(radians(T2))) / (cos(radians(latitude))*sin(radians(AD2))))) + 180) % 360);
	else return (double)((long)(540 - degrees(acos(((sin(radians(latitude))*cos(radians(AD2))) - sin(radians(T2))) / (cos(radians(latitude))*sin(radians(AD2)))))) % 360);
}

int set_servo_azimuth(int angle) //-90* - 90*
{
	int pwm_value = 1500 + angle*10;

	if(angle < -90)
	{
		pwm_value = 600;
	}
	else if(angle > 90)
	{
		pwm_value = 2400;
	}
	else
	{
		pwm_value = 1500 + angle*10;
	}

	return pwm_value;
}

int set_servo_elevation(int angle) //0* - 85*
{
	int pwm_value;

	if(angle < 0)
	{
		pwm_value = 1500;
	}
	else if(angle > 85)
	{
		pwm_value = 2350;
	}
	else
	{
		pwm_value = 1500 + angle*10;
	}

	return pwm_value;
}

void presentation_mode()
{
	if(presentation_mode_counter <= 1)
	{
		int azimuth = (int)solar_azimuth_angle(2, 52.24, 16.55, presentation_mode_counter, 43636);
		int elevation = (int)solar_elevation_angle(2, 52.24, 16.55, presentation_mode_counter, 43636);
		if(elevation > 0)
		{
			AZIMUTH = set_servo_azimuth((azimuth - 180)*(-1));
			ELEVATION = set_servo_elevation(90 - elevation);
		}
		else
		{
			AZIMUTH = set_servo_azimuth(0);
			ELEVATION = set_servo_elevation(0);
		}

		presentation_mode_counter+=0.0005;
	}
	else
	{
		presentation_mode_counter = 0;
	}
}

double part_of_day(double hour, double minute, double second)
{
	return (hour*3600+minute*60+second)/86400;
}

int days_since_1900(int year, int month, int day)
{
	int days = 0;
	int y = year - 1900;
	days += y * 365 + y / 4;
	switch (month)
	{
	case 12: days += 30;
	case 11: days += 31;
	case 10: days += 30;
	case 9: days += 31;
	case 8: days += 31;
	case 7: days += 30;
	case 6: days += 31;
	case 5: days += 30;
	case 4: days += 31;
	case 3: days += 28;
	case 2: days += 31;
	}
	days += day;

	return days - 1;
}

void normal_mode()
{
	HAL_UART_Receive_IT(&huart4, receiveUART, sizeReceiveUART);
	int repeat_couter = 10;
	int is_gps_good = 0;

	if(parse_input(receiveUART, sizeReceiveUART, &GPS) == 0)
	{
		AZIMUTH = set_servo_azimuth(0);
		ELEVATION = set_servo_elevation(50);
	}
	else is_gps_good = 1;

	while(is_gps_good == 0 && repeat_couter > 0)
	{
		HAL_UART_Receive_IT(&huart4, receiveUART, sizeReceiveUART);
		is_gps_good = parse_input(receiveUART, sizeReceiveUART, &GPS);
		repeat_couter--;
	}

	if(is_gps_good == 1)
	{
		double part = part_of_day((double)GPS.hour, (double)GPS.minute, (double)GPS.second);
		int days = days_since_1900((int)GPS.year, (int)GPS.month, (int)GPS.day);

		int azmth = solar_azimuth_angle(2, GPS.latitude, GPS.longitude, part, days);
		int elvtn = solar_azimuth_angle(2, GPS.latitude, GPS.longitude, part, days);

		AZIMUTH = set_servo_azimuth((azmth-180)*(-1));
		ELEVATION = set_servo_elevation(90 - elvtn);
	}
}


#endif /* TRACKER_H_ */
