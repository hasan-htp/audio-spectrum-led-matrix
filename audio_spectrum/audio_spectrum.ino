#include <arduinoFFT.h>
#include <math.h>
#include <MD_MAX72xx.h>
#include <MD_Parola.h>

const int samplingFrequency = 20000;
const int sampleCount = 512;
const int ledColoumncount = 32;
double vReal[sampleCount];
double vImag[sampleCount];

const auto micInputPin = 35;
const float dcOfffset = 1.25; //volt
const float vrefDB = 0.775;

ArduinoFFT<double> fft = ArduinoFFT<double>();
MD_MAX72XX ledMatrix = MD_MAX72XX(MD_MAX72XX::FC16_HW, 21, 4); //cs pin 21, 4 devices

const uint16_t freqs[ledColoumncount+1] = {
                                            1,2,3,4,5,6,7,8, // 39hz ++39
                                            9,10,11,12,13,14,15,16, // 351hz ++39
                                            17,18,19,20,21,22,23,24, // 663hz ++39
                                            25,35,50,70,95,125,160,200, // 975hz, 1365, 1950, 2730, 3705, 4875, 6240, 7800
                                            255                         //9945hz (for calculating avgs)
};

const float gains[ledColoumncount] = {0.4, 0.4, 0.5, 0.5, 0.6, 0.6, 0.7, 0.7,
                                     0.8, 0.8, 0.9, 0.9, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1,
                                     1, 1, 1, 1, 1, 1, 1, 1
                                     };

int displayValues[]={0, 1, 3, 7, 15, 31, 63, 127, 255}; 
const float freqFac = 39; // 0.5* 200000/128

const float dynamicGains[8] = {0.25, 0.5, 0.75, 1, 1.25, 1.5, 1.75, 2};
uint8_t gainIndex = 3;
volatile bool interrupt = false;

void SerialPrint(int freq, int mag)
{
    Serial.print("Frequency: ");
    Serial.print(freq*freqFac);
    Serial.print(" Hz, Magnitude: ");
    Serial.println(mag);
}

void CheckGainInput() {
  interrupt=true;
}

void ProccessGainInput(){
    gainIndex++;
    if(gainIndex>7){
      gainIndex=0;
    }
    for (int i = 0; i <ledColoumncount; i++) {
      ledMatrix.setColumn(i,displayValues[gainIndex+1]);// to show gain level (show from 1 to 8 dots)
    }
}

double Avg(double * array, uint32_t size)
{
  double avg = 0;
  for(uint32_t i=0; i<size; i++){
    avg += array[i]/(double)size;
  }
  return avg;
}

void ShowSpectrum(){
  for (int i = 0; i < ledColoumncount; i++) {
    const auto windowSizeOfAvg = freqs[i+1]- freqs[i];
    double val = Avg(&vReal[freqs[i]], windowSizeOfAvg);
    val *=  gains[i] *  dynamicGains[gainIndex];
    //SerialPrint(freqs[i],val);
    
    float limitedVal = val > 8.0 ? 8.0 : val;
    limitedVal = limitedVal < 0.5 ? 0 : limitedVal;

    int displayVal = nearbyintf(limitedVal);
    ledMatrix.setColumn(i,displayValues[displayVal]);
  }
  //Serial.println("--------");
}

void setup() {
  delay(1000);
  ledMatrix.begin(); 
  delay(1000);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), CheckGainInput, FALLING);
  //Serial.begin(115200);
}

void loop() {
  if(interrupt){
    ProccessGainInput();
    delay(500);
    interrupt = false;
  }

  for (int i = 0; i < sampleCount; i++) {
    auto startMicros = micros();
    const int adcValue = analogRead(micInputPin);
    const float voltage = ((adcValue * 3.3) / 4096.0) -dcOfffset;
    const float correctedVoltage = voltage > 0 ? voltage : 0;

    vReal[i] = correctedVoltage;
    vImag[i] = 0;
    while (micros() - startMicros < (1000000/samplingFrequency));
  }

  fft.windowing(vReal, sampleCount, FFTWindow::Hamming, FFTDirection::Forward);
  fft.compute(vReal, vImag, sampleCount,FFTDirection::Forward);
  fft.complexToMagnitude(vReal, vImag, sampleCount);

  ShowSpectrum();
}
