
uint32_t adc_val;
double vOUT = 0.0;
double vIN = 0.0;
//double R1 = 100000.0;
//double R2 = 10000.0;

float R1 = 30000.0;
float R2 = 7500.0;

// Constante pentru filtrul EMA
const float alpha = 0.001; // Factorul de atenuare (0 < alpha < 1)

float vIN_filtered = 0.0; // Variabila pentru voltajul filtrat
// Definirea dimensiunii bufferului circular
#define bufferSize  100 // Numărul de eșantioane pe care le vom media

unsigned long values[bufferSize]; // Array pentru stocarea valorilor citite
int index2 = 0; // Indexul curent în bufferul circular
unsigned long sum = 0; // Variabila pentru stocarea sumei eșantioanelor

float lastVoltage = 0; // Variabila pentru stocarea ultimei tensiuni citite
float thresholdDerivative = 0.02; // Pragul derivatului pentru a detecta schimbările semnificative (modificați după caz)
float  V_filtrat;

void voltmetru() {

    HAL_ADC_Start_IT(&hadc1);
      adc_val = HAL_ADC_GetValue(&hadc1);


    sum = sum - values[index2] + adc_val; // Actualizarea sumei
    values[index2] = adc_val; // Actualizarea valorii din buffer
    index2 = (index2 + 1) % bufferSize; // Incrementarea indexului circular

    vOUT = sum / (float)bufferSize * (3.33 / 4095.0);
    vIN = vOUT / (R2/(R1+R2));

    // Filtrarea voltajului de intrare (vIN) utilizând filtrul EMA
    V_filtrat = (alpha * vIN) + ((1 - alpha) * V_filtrat);

    // Calcularea derivatelor dintre valori
    float derivative = vIN_filtered - lastVoltage;

    // Verificare dacă derivata depășește pragul
    if (derivative < 0) {
        derivative = -derivative;
    }
    if (derivative > thresholdDerivative) {
        vIN_filtered = V_filtrat;
    }

    // Actualizarea ultimei tensiuni citite
    lastVoltage = V_filtrat;

}

