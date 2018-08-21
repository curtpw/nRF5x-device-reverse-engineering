#ifndef FIR_h
#define FIR_h

template <typename T, int ntaps>
class FIR {
public:
  FIR();
  void setGain(T newgain);
  T getGain();
  void setFilterCoeffs(T *coeffs);
  T processReading(T newval);

private:
  T values[ntaps];
  T fir_coeffs[ntaps];
  T gain;
  int k;
};

#include "FIR.tpp"
#endif
