#ifndef INC_FUNCTII_GPS_H_
#define INC_FUNCTII_GPS_H_

#include <stdint.h>



#define RADIUS 6372795.0
#define PI 3.1415926535897932384626433832795


double distanceBetween(double lat1, double long1, double lat2, double long2);
double courseTo(double lat1, double long1, double lat2, double long2);
float calculateHeadingError(float desiredBearing, float currentHeading);



// Structura pentru filtrul Kalman
typedef struct {
    double lat; // Estimarea latitudinii
    double lon; // Estimarea longitudinii
    double lat_variance; // Varianța latitudinii
    double lon_variance; // Varianța longitudinii
} KalmanFilterGPS;

//extern KalmanFilterGPS kalmanFilter;

KalmanFilterGPS initKalmanFilter(double lat, double lon, double lat_variance, double lon_variance);
void updateKalmanFilter(KalmanFilterGPS *filter, double lat_measurement, double lon_measurement, double time_difference) ;

//double time_difference = 1.0; // Diferența de timp între măsurători (în ore)
double calculateSpeed(double lat1, double long1, double lat2, double long2, double time_difference);


typedef struct {
    float kp;           // Coeficientul proportional
    float ki;           // Coeficientul integral
    float kd;           // Coeficientul derivative
    float setpoint;     // Valoarea dorită (target)
    float last_error;   // Ultima eroare
    float integral;     // Suma erorilor
    float min_output;   // Limita inferioară a valorii de ieșire
    float max_output;   // Limita superioară a valorii de ieșire
    uint8_t disabled;
    uint8_t integral_active; // Marchează dacă integrala PID este activă sau nu
    uint8_t reverse_direction;

} PIDController;

void initPID(PIDController *pid, float kp, float ki, float kd, float setpoint, float min_output, float max_output);
float computePID(PIDController *pid, float current_value);

// Funcție pentru calculul ieșirii PID
//float computePID(PIDController *pid, float current_value) ;


//int error;


#endif /* INC_FUNCTII_GPS_H_ */
