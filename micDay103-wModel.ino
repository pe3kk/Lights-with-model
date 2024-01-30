#include <cleaanData_inferencing.h>
#include <Audio.h>
#include <FastLED.h>
#include <deque>
#include <AudioStream.h>

#define BUFFER_SIZE 4410

#define NUM_LEDS  120
#define NUM_LEDS_T_and_B 36
#define NUM_LEDS_MID 120

#define LED_PIN0   33
#define CLOCK_PIN  29

//PULSE
#define SCROLL_SPEED 1
unsigned long lastUpdate;

CRGB leds0[NUM_LEDS_MID];

//const int myInput = AUDIO_INPUT_MIC;
const int myInput = AUDIO_INPUT_LINEIN;
AudioInputI2S          audioInput;
AudioMixer4            mixer;
AudioAnalyzeFFT1024    fft;
AudioOutputI2S         audioOutput;

AudioRecordQueue       queue;
AudioFilterBiquad      lowpassFilter;

// AudioConnection myConnection(source, sourcePort, destination, destinationPort);
AudioConnection patchCord1(audioInput, 0, mixer, 0);
AudioConnection patchCord2(audioInput, 1, mixer, 1);
AudioConnection patchCord3(mixer, fft);             // using mixer for stereo line-in connection

AudioConnection patchCord4(mixer, 0, lowpassFilter, 0);
AudioConnection patchCord5(lowpassFilter, 0, queue, 0);

//AudioConnection patchCord1(sinewave, 0, myFFT, 0);
AudioControlSGTL5000 audioShield;

const unsigned int arraySize = 250;                         //related to deque container
const unsigned int arraySizeSh = 50;                        //related to deque container

int newVal = 0;                                     //related to potentiometers
int val = 0;                                        //related to potentiometers
int k = 0;                                          //variable switches subArray if colorArrays1 is in use

    float subbass = 0, all = 0;
    float maxSubBassVal = 0, maxAllVal = 0;
    float avgBassKick;

float subBassAvg, allAvg;
int subBassAvgCnt = 0, allAvgCnt;
float subBassAvgOut = 0, allAvgOut = 0;

float subBassAvgSh, allAvgSh;
int subBassAvgCntSh = 0, allAvgCntSh = 0;
float subBassAvgOutSh = 0, allAvgOutSh = 0;

int subbassBh = 0, allBh = 0;
int subbassCnt = 0, allCnt = 0;
int subBassBin = 0, allBin = 0;

int fadeAmount = 1;
int color = 0;
int mode = 0;

float maxBassKick;
int maxBassBin;

float audioBuffer[BUFFER_SIZE];
int bufferIdx = 0;
float resultVal;

int allBrigh;
bool kickFlag = 0;

const int maxBrigh = 50;

void setup() {
  FastLED.addLeds<APA102, LED_PIN0, CLOCK_PIN,  BGR>(leds0, NUM_LEDS_MID);
  FastLED.setBrightness(220);
  
  queue.begin();
  lowpassFilter.setLowpass(0, 200, 0.707);

  AudioMemory(30);
  audioShield.enable();
  audioShield.inputSelect(myInput);
  audioShield.lineInLevel(7);
  //audioShield.micGain(15);
  fft.windowFunction(AudioWindowBlackman1024);
  Serial.begin(115200);
}

void processBuffer() {
    // Ensure the buffer size is correct
    if (BUFFER_SIZE != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        Serial.printf("The size of your 'audioBuffer' array is not correct. Expected %lu items, but had %lu\n",
                      EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, BUFFER_SIZE);
        return;
    }

    // Perform inference
    signal_t features_signal;
    features_signal.total_length = BUFFER_SIZE;
    features_signal.get_data = &raw_feature_get_data;
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false);
    
    if (res != EI_IMPULSE_OK) {
        Serial.printf("ERR: Failed to run classifier (%d)\n", res);
        return;
    }
    
    // Output the result
    print_inference_result(result);
}

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, audioBuffer + offset, length * sizeof(float));
    return 0;
}

void print_inference_result(ei_impulse_result_t result) {

    // Print how long it took to perform inference
    ei_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n",
            result.timing.dsp,
            result.timing.classification,
            result.timing.anomaly);

    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
        resultVal = result.classification[0].value;
    }

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif
}

void audioBufferFun() {
  if (queue.available()) {
      // Read the audio data into the buffer
      int16_t *buffer = queue.readBuffer();
      for (int j = 0; j < AUDIO_BLOCK_SAMPLES; j++) {
        audioBuffer[bufferIdx] = (float)buffer[j];
        bufferIdx++;
        if (bufferIdx >= BUFFER_SIZE) {
          processBuffer();
          bufferIdx = 0;
        }
      }
        queue.freeBuffer(); // Free the buffer after processing
     }
}

class Leds {
private:
  int numLeds;
  CRGB* leds;
  
public:
  Leds(int initialNumLeds, CRGB* initialLeds)
    : numLeds(initialNumLeds), leds(initialLeds)
  {
    
  }

  void moveLnR()  {
    // Shift LEDs from the middle to the right
    for (int i = numLeds - 1; i > numLeds / 2; i--)
        leds[i] = leds[i - 1];
  
    // Shift LEDs from the middle to the left
    for (int j = 0; j < numLeds / 2; j++)
        leds[j] = leds[j + 1];
  }

  void updateLEDs(float val, float valAvgOut, float maxVal, float threshold, int& brightness, int colorIndex) {
    switch (mode) {
      case 0:
        if (maxVal > maxBassKick * 0.65 && maxVal > avgBassKick * 0.65 && resultVal >= 0.5) {
          kickFlag = 1;
          brightness = 255;
          leds[numLeds / 2] = CHSV(220, 255, brightness);
          moveLnR();
        } else {
          kickFlag = 0;
          if (all >= allAvgOut * 1.5)  {
            allBrigh += maxBrigh / 10;
          } else {
            allBrigh -= maxBrigh / 10;
          }
          if (allBrigh <= 20) {
            allBrigh = 20;
          }
          else if (allBrigh >= 254) {
            allBrigh  = 250;
          }
            leds[numLeds / 2] = CHSV(160, 255, allBrigh); // Black color
            moveLnR();
        }
        break;
    }
  }
};

class Buffer {
private:
  const unsigned int arraySize = 250;
  const unsigned int arraySizeSh = 50;
  int avgOut;
  std::deque<float> buffer;            // Added deque for buffer

public:
  Buffer(int initialBufferSize)
    : arraySize(initialBufferSize),
      buffer(initialBufferSize) {  // Initialize buffer with initialBufferSize
    buffer.resize(arraySize);
  }

  float movingAverage(const std::deque<float>& buffer, int arrSz) {
    float sum = 0;
    for (float value : buffer) {
      sum += value;
    }
    return sum / arrSz;
  }

  void updateMovingAverages(float& output, int arrSz) {
    if (buffer.size() >= arrSz) {
      output = movingAverage(buffer, arrSz);
    }
  }

  void updateBuffer(float newestBufferVal, float& avgOut, int arrSz) {
    buffer.pop_front();
    buffer.push_back(newestBufferVal);
    updateMovingAverages(avgOut, arrSz);
  }
 
};

    Leds ledStrip0(NUM_LEDS_MID, leds0);

    Buffer subbassBuffer(arraySize);
    Buffer subbassBufferSh(arraySizeSh);
    Buffer allBuffer(arraySize);
    Buffer allBufferSh(arraySizeSh);   
     
    Buffer maxBassKickBuff(arraySize);

void FFTnLeds() {
  float n = 0;
  int i = 0;
  
  if (fft.available()) {
      subbass = 0, all = 0;
      maxSubBassVal = 0, maxAllVal = 0;
      subBassBin = 0;
  
      
      for (i = 0; i < 200; i++) {
        n = fft.read(i);
  
        if (i >= 3 && i <= 5) {
          if (n >= maxSubBassVal) {
            maxSubBassVal = n;
            subBassBin = i;
          }
          if (maxSubBassVal > maxBassKick) {
            maxBassKick = maxSubBassVal * 0.9;
            maxBassBin = subBassBin;
          } else  {
            maxBassKick -= 0.0001;
          }
          subbass += n;
        }
        else if (i >= 0 && i <= 200)  {
          all += n;
        }
      }
  
      subbassBuffer.updateBuffer(subbass, subBassAvgOut, arraySize);
      subbassBufferSh.updateBuffer(subbass, subBassAvgOutSh, arraySizeSh);
      allBuffer.updateBuffer(all, allAvgOut, arraySize);
      allBufferSh.updateBuffer(all, allAvgOutSh, arraySizeSh);
      maxBassKickBuff.updateBuffer(maxBassKick, avgBassKick, arraySize);
      
      ledStrip0.updateLEDs(subbass, subBassAvgOut, maxSubBassVal, 0.03, subbassBh, subBassBin);
      FastLED.show();
    }
  }

void loop() {
  audioBufferFun();
  FFTnLeds();
}
