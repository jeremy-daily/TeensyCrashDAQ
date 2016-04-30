#ifndef UserDataType_h
#define UserDataType_h
const uint8_t ADC_DIM = 2;
struct data_t {
  uint32_t time;
  uint16_t adc[ADC_DIM];
};
const uint8_t readPins[ADC_DIM] = {A10,A11};

#endif  // UserDataType_h
