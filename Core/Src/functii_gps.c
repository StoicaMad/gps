#include "functii_gps.h"

#include <math.h>


// Funcția care calculează distanța în metri între două puncte de coordonate GPS
double distanceBetween(double lat1, double long1, double lat2, double long2) {
    // Convertim gradele în radiani
    double lat1_rad = lat1 * M_PI / 180.0;
    double long1_rad = long1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double long2_rad = long2 * M_PI / 180.0;

    // Calculăm diferența de longitudine
    double delta_long = long1_rad - long2_rad;

    // Calculăm sinusul și cosinusul diferenței de longitudine
    double sin_delta_long = sin(delta_long);
    double cos_delta_long = cos(delta_long);

    // Calculăm sinusul și cosinusul latitudinilor
    double sin_lat1 = sin(lat1_rad);
    double sin_lat2 = sin(lat2_rad);
    double cos_lat1 = cos(lat1_rad);
    double cos_lat2 = cos(lat2_rad);

    // Calculăm diferența de longitudine dintre cele două puncte
    double delta = (cos_lat1 * sin_lat2) - (sin_lat1 * cos_lat2 * cos_delta_long);
    delta = pow(delta, 2); // Ridicăm la pătrat
    delta += pow(cos_lat2 * sin_delta_long, 2); // Adăugăm pătratul diferenței de longitudine
    delta = sqrt(delta); // Calculăm rădăcina pătrată

    // Calculăm denumitorul
    double denom = (sin_lat1 * sin_lat2) + (cos_lat1 * cos_lat2 * cos_delta_long);

    // Calculăm delta finală
    delta = atan2(delta, denom);

    // Întoarcem distanța în metri
    return delta * RADIUS;
}



double radians(double deg) {
    return deg * (PI / 180.0);
}

double degrees(double rad) {
    return rad * (180.0 / PI);
}

double courseTo(double lat1, double long1, double lat2, double long2) {
    // returns course in degrees (North=0, West=270) from position 1 to position 2,
    // both specified as signed decimal-degrees latitude and longitude.
    // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
    // Courtesy of Maarten Lamers
    double dlon = radians(long2 - long1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double a1 = sin(dlon) * cos(lat2);
    double a2 = sin(lat1) * cos(lat2) * cos(dlon);
    a2 = cos(lat1) * sin(lat2) - a2;
    a2 = atan2(a1, a2);
    if (a2 < 0.0) {
        a2 += 2 * PI;
    }
    return degrees(a2);
}



float calculateHeadingError(float desiredBearing, float currentHeading) {
    float error = desiredBearing - currentHeading;

    // Ajustează eroarea pentru cea mai scurtă distanță
    if (error > 180) {
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    return error;
}



// Funcție pentru inițializarea filtrului Kalman
KalmanFilterGPS initKalmanFilter(double lat, double lon, double lat_variance, double lon_variance) {
    KalmanFilterGPS filter;
    filter.lat = lat;
    filter.lon = lon;
    filter.lat_variance = lat_variance;
    filter.lon_variance = lon_variance;
    return filter;
}



// Funcție pentru actualizarea filtrului Kalman cu o nouă măsurătoare GPS
void updateKalmanFilter(KalmanFilterGPS *filter, double lat_measurement, double lon_measurement, double time_difference) {
    // Procesul de predicție
    double process_variance = 0.1; // Variabilitatea procesului (modificați după caz)
    filter->lat_variance += process_variance * time_difference;
    filter->lon_variance += process_variance * time_difference;

    // Actualizarea stării curente cu măsurătoarea GPS
    double innovation_lat = lat_measurement - filter->lat;
    double innovation_lon = lon_measurement - filter->lon;

    // Calcularea ponderilor Kalman (incredere acordata datelor gps)
    double gain_lat = filter->lat_variance / (filter->lat_variance + 1.0); // Modificați covarianța măsurării GPS
    double gain_lon = filter->lon_variance / (filter->lon_variance + 1.0); // Modificați covarianța măsurării GPS

    // Actualizarea estimării stării
    filter->lat += gain_lat * innovation_lat;
    filter->lon += gain_lon * innovation_lon;

    // Actualizarea varianței estimării
    filter->lat_variance *= (1.0 - gain_lat);
    filter->lon_variance *= (1.0 - gain_lon);
}


double calculateSpeed(double lat1, double long1, double lat2, double long2, double time_difference) {
    // Calculați distanța între cele două puncte
    double distance = distanceBetween(lat1, long1, lat2, long2);

    // Calculați viteza ca raport între distanță și timp
    double speed = distance / time_difference;

    // Convertiți viteza din metri pe secundă în kilometri pe oră
    speed *= 3.6; // 1 m/s = 3.6 km/h

    return speed;
}

void initPID(PIDController *pid, float kp, float ki, float kd, float setpoint, float min_output, float max_output) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;
    pid->last_error = 0.0;
    pid->integral = 0.0;
    pid->min_output = min_output;
    pid->max_output = max_output;
}
float computePID(PIDController *pid, float current_value) {
    // Verificați dacă PID-ul trebuie să fie activ sau nu
    if (!pid->disabled) {
        // Calculați eroarea curentă
        float error = pid->setpoint - current_value;

        // Calculați termenii PID
        float proportional = pid->kp * error;

        // Dacă PID-ul este activat, resetăm integrala la 0
        if (!pid->integral_active) {
            pid->integral = 0.0f;
            pid->integral_active = 1; // Marcați integrala ca activă pentru următoarea iterație
        }
        pid->integral += pid->ki * error;

        // Limitați termenul integral pentru a preveni integral windup
        if (pid->integral > pid->max_output) {
            pid->integral = pid->max_output;
        } else if (pid->integral < pid->min_output) {
            pid->integral = pid->min_output;
        }

        float derivative = pid->kd * (error - pid->last_error);

        // Actualizați eroarea anterioară
        pid->last_error = error;

        // Calculați valoarea de control PID
        float control = proportional + pid->integral + derivative;

        // Inversarea direcției controlului PID dacă este necesar
        if (pid->reverse_direction) {
            control = -control;
            float temp = pid->min_output;
            pid->min_output = -pid->max_output;
            pid->max_output = -temp;
        }

        // Limitați valoarea de control în intervalul [min_output, max_output]
        if (control < pid->min_output) {
            control = pid->min_output;
        } else if (control > pid->max_output) {
            control = pid->max_output;
        }

        return control;
    } else {
        // În cazul în care PID-ul este dezactivat, resetăm integrala ca neactivată
        pid->integral_active = 0;
        // În cazul în care PID-ul este dezactivat, returnați 0
        return 0.0f;
    }
}

/*
float computePID(PIDController *pid, float current_value) {
    // Verificați dacă PID-ul trebuie să fie activ sau nu
    if (!pid->disabled) {
        // Calculați eroarea curentă
        float error = pid->setpoint - current_value;

        // Calculați termenii PID
        float proportional = pid->kp * error;

        // Dacă PID-ul este activat, resetăm integrala la 0
        if (!pid->integral_active) {
            pid->integral = 0.0f;
            pid->integral_active = 1; // Marcați integrala ca activă pentru următoarea iterație
        }
        pid->integral += pid->ki * error;

        // Limitați termenul integral pentru a preveni integral windup
        if (pid->integral > pid->max_output) {
            pid->integral = pid->max_output;
        } else if (pid->integral < pid->min_output) {
            pid->integral = pid->min_output;
        }

        float derivative = pid->kd * (error - pid->last_error);

        // Actualizați eroarea anterioară
        pid->last_error = error;

        // Calculați valoarea de control PID
        float control = proportional + pid->integral + derivative;

        // Limitați valoarea de control în intervalul [min_output, max_output]
        if (control < pid->min_output) {
            control = pid->min_output;
        } else if (control > pid->max_output) {
            control = pid->max_output;
        }

        return control;
    } else {
        // În cazul în care PID-ul este dezactivat, resetăm integrala ca neactivată
        pid->integral_active = 0;
        // În cazul în care PID-ul este dezactivat, returnați 0
        return 0.0f;
    }
}
*/
