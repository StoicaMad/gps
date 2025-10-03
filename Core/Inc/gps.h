/*
 * gps.h
 *
 *  Created on: Nov 15, 2019
 *      Author: Bulanov Konstantin
 */

#ifndef GPS_H
#define GPS_H

#include <stdint.h>


#define GPS_DEBUG	0
#define	GPS_USART	&huart3
#define GPSBUFSIZE  128       // GPS buffer size

typedef struct{

    // calculated values
    float dec_longitude;
    float dec_latitude;
    float altitude_ft;

    // GGA - Global Positioning System Fixed Data
    float nmea_longitude;
    float nmea_latitude;
    float utc_time;
    char ns, ew;
    int lock;
    int satelites;
    float hdop;
    float msl_altitude;
    char msl_units;

    // RMC - Recommended Minimmum Specific GNS Data
    char rmc_status;
    float speed_k;
    float course_d;
    int date;
    char status;


    // GLL
    char gll_status;

    // VTG - Course over ground, ground speed
    float course_t; // ground speed true
    char course_t_unit;
    float course_m; // magnetic
    char course_m_unit;
        float speed_k_2;
    char speed_k_unit;
    float speed_km; // speek km/hr
    char speed_km_unit;



    float courseT;       // Cursul dorit, în grade
    char refT;           // Referința cursului dorit (N = nord, S = sud)
    float courseM;       // Cursul magnetic, în grade
    char refM;           // Referința cursului magnetic (E = est, W = vest)
    float speedK;        // Viteza terestră în noduri
    char unitsK;         // Unitatea pentru viteza terestră (N = noduri)
    float speedM;        // Viteza terestră în kilometri pe oră
    char unitsM;         // Unitatea pentru viteza terestră (K = kilometri pe oră)

    float nmea_longitude_2;
    float nmea_latitude_2;
    float utc_time_2;
} GPS_t;

extern GPS_t GPS;


#if (GPS_DEBUG == 1)
void GPS_print(char *data);
#endif

void GPS_Init();
void GSP_USBPrint(char *data);
void GPS_print_val(char *data, int value);
void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);


#endif /* GPS_H */
