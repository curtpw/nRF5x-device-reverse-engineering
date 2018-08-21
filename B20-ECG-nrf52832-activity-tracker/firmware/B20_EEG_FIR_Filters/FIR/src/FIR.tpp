template <typename T, int ntaps>
FIR<T, ntaps>::FIR() {
  k = 0;
  memset(values, 0, sizeof(values));
}

template <typename T, int ntaps>
void FIR<T, ntaps>::setGain(T newgain) {
  gain = newgain;
}

template <typename T, int ntaps>
T FIR<T, ntaps>::getGain() {
  return gain;
}

template <typename T, int ntaps>
void FIR<T, ntaps>::setFilterCoeffs(T *coeffs) {
  memcpy(&fir_coeffs, coeffs, sizeof(fir_coeffs));

  // Automatically calculate the gain needed by sending unity inputs for at
  // least the legnth of the filter.
  T input = 1;
  T new_gain;
  gain = 1;
  for (int i=0; i<ntaps; i++) {
    new_gain = FIR<T, ntaps>::processReading(input);
  }

  // Cleanup by zeroing out the data and index for the filter
  k = 0;
  for (int i=0; i<ntaps; i++){
    values[i] = 0;
  }

  FIR<T, ntaps>::setGain(new_gain); // Set the new gain
}

template <typename T, int ntaps>
T FIR<T, ntaps>::processReading(T newval){
  T output = 0;
  values[k] = newval;

  for (int i=0; i<ntaps; i++){
    output += fir_coeffs[i] * values[(i + k) % ntaps];
  }

  k = (k+1) % ntaps;

  return output/gain;
}
