#include <Arduino.h>
#include <arduinoFFT.h>

const int micPin = 2; // Microphone pin
const int sampleRate = 4000; // Sample rate in Hz
const int sampleSize = 256; // Number of samples for FFT
unsigned int samplingPeriod;
unsigned long microSeconds;

double vReal[sampleSize];
double vImag[sampleSize];
arduinoFFT FFT = arduinoFFT(vReal, vImag, sampleSize, sampleRate);

void setup() {
  Serial.begin(115200);
  pinMode(micPin, INPUT);
  samplingPeriod = round(1000000*(1.0/sampleRate));
}

void loop() {
  // Read the input from the microphone
  for(int i=0; i<sampleSize; i++)
  {
    microSeconds = micros(); // Overhead of calling micros() can be significant
    vReal[i] = analogRead(micPin);
    vImag[i] = 0;
    while(micros() < (microSeconds + samplingPeriod)){
      // Wait until the sampling period is reached
    }
  }

  // Perform FFT
  FFT.Windowing(vReal, sampleSize, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // Find the dominant frequency
  double peakFrequency = 0;
  int peakBin = FFT.MajorPeak();

  peakFrequency = (double)peakBin * (double)sampleRate / (double)sampleSize;
  Serial.print("Peak Frequency: ");
  Serial.println(peakFrequency);

  // Simple sound level measurement (peak-to-peak)
  double soundLevel = 0;
  for(int i=0; i<sampleSize; i++) {
    soundLevel += vReal[i]; // Summing up for an approximation
  }
  soundLevel /= sampleSize; // Average amplitude
  Serial.print("Sound Level: ");
  Serial.println(soundLevel);

  delay(1000); // Delay between measurements
}
