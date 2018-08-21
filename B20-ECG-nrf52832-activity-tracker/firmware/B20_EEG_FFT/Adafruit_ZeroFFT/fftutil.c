/*
 * fftutil.c
 *
 *  Created on: Jan 5, 2018
 *      Author: deanm
 */

#include "Adafruit_ZeroFFT.h"

/*
   * @brief  In-place bit reversal function.
   * @param[in, out] *pSrc        points to the in-place buffer of Q15 data type.
   * @param[in]      fftLen       length of the FFT.
   * @param[in]      bitRevFactor bit reversal modifier that supports different size FFTs with the same bit reversal table
   * @param[in]      *pBitRevTab  points to bit reversal table.
   * @return none.
 */

static q15_t scratchData[ZERO_FFT_MAX];

void arm_bitreversal_q15(
  q15_t * pSrc16,
  uint32_t fftLen,
  uint16_t bitRevFactor,
  uint16_t * pBitRevTab)
{
  q31_t *pSrc = (q31_t *) pSrc16;
  q31_t in;
  uint32_t fftLenBy2, fftLenBy2p1;
  uint32_t i, j;

  /*  Initializations */
  j = 0u;
  fftLenBy2 = fftLen / 2u;
  fftLenBy2p1 = (fftLen / 2u) + 1u;

  /* Bit Reversal Implementation */
  for (i = 0u; i <= (fftLenBy2 - 2u); i += 2u)
  {
    if(i < j)
    {
      /*  pSrc[i] <-> pSrc[j]; */
      /*  pSrc[i+1u] <-> pSrc[j+1u] */
      in = pSrc[i];
      pSrc[i] = pSrc[j];
      pSrc[j] = in;

      /*  pSrc[i + fftLenBy2p1] <-> pSrc[j + fftLenBy2p1];  */
      /*  pSrc[i + fftLenBy2p1+1u] <-> pSrc[j + fftLenBy2p1+1u] */
      in = pSrc[i + fftLenBy2p1];
      pSrc[i + fftLenBy2p1] = pSrc[j + fftLenBy2p1];
      pSrc[j + fftLenBy2p1] = in;
    }

    /*  pSrc[i+1u] <-> pSrc[j+fftLenBy2];         */
    /*  pSrc[i+2] <-> pSrc[j+fftLenBy2+1u]  */
    in = pSrc[i + 1u];
    pSrc[i + 1u] = pSrc[j + fftLenBy2];
    pSrc[j + fftLenBy2] = in;

    /*  Reading the index for the bit reversal */
    j = *pBitRevTab;

    /*  Updating the bit reversal index depending on the fft length  */
    pBitRevTab += bitRevFactor;
  }
}

void arm_radix2_butterfly_q15(
  q15_t * pSrc,
  uint32_t fftLen,
  q15_t * pCoef,
  uint16_t twidCoefModifier)
{
  int i, j, k, l;
  int n1, n2, ia;
  q15_t xt, yt, cosVal, sinVal;


  //N = fftLen;
  n2 = fftLen;

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  // loop for groups
  for (j = 0; j < n2; j++)
  {
    cosVal = pCoef[ia * 2];
    sinVal = pCoef[(ia * 2) + 1];
    ia = ia + twidCoefModifier;

    // loop for butterfly
    for (i = j; i < fftLen; i += n1)
    {
      l = i + n2;
      xt = (pSrc[2 * i] >> 2u) - (pSrc[2 * l] >> 2u);
      pSrc[2 * i] = ((pSrc[2 * i] >> 2u) + (pSrc[2 * l] >> 2u)) >> 1u;

      yt = (pSrc[2 * i + 1] >> 2u) - (pSrc[2 * l + 1] >> 2u);
      pSrc[2 * i + 1] =
        ((pSrc[2 * l + 1] >> 2u) + (pSrc[2 * i + 1] >> 2u)) >> 1u;

      pSrc[2u * l] = (((int16_t) (((q31_t) xt * cosVal) >> 16)) +
                      ((int16_t) (((q31_t) yt * sinVal) >> 16)));

      pSrc[2u * l + 1u] = (((int16_t) (((q31_t) yt * cosVal) >> 16)) -
                           ((int16_t) (((q31_t) xt * sinVal) >> 16)));

    }                           // butterfly loop end

  }                             // groups loop end

  twidCoefModifier = twidCoefModifier << 1u;

  // loop for stage
  for (k = fftLen / 2; k > 2; k = k >> 1)
  {
    n1 = n2;
    n2 = n2 >> 1;
    ia = 0;

    // loop for groups
    for (j = 0; j < n2; j++)
    {
      cosVal = pCoef[ia * 2];
      sinVal = pCoef[(ia * 2) + 1];
      ia = ia + twidCoefModifier;

      // loop for butterfly
      for (i = j; i < fftLen; i += n1)
      {
        l = i + n2;
        xt = pSrc[2 * i] - pSrc[2 * l];
        pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]) >> 1u;

        yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
        pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]) >> 1u;

        pSrc[2u * l] = (((int16_t) (((q31_t) xt * cosVal) >> 16)) +
                        ((int16_t) (((q31_t) yt * sinVal) >> 16)));

        pSrc[2u * l + 1u] = (((int16_t) (((q31_t) yt * cosVal) >> 16)) -
                             ((int16_t) (((q31_t) xt * sinVal) >> 16)));

      }                         // butterfly loop end

    }                           // groups loop end

    twidCoefModifier = twidCoefModifier << 1u;
  }                             // stages loop end

  n1 = n2;
  n2 = n2 >> 1;
  ia = 0;

  // loop for groups
  for (j = 0; j < n2; j++)
  {
    cosVal = pCoef[ia * 2];
    sinVal = pCoef[(ia * 2) + 1];

    ia = ia + twidCoefModifier;

    // loop for butterfly
    for (i = j; i < fftLen; i += n1)
    {
      l = i + n2;
      xt = pSrc[2 * i] - pSrc[2 * l];
      pSrc[2 * i] = (pSrc[2 * i] + pSrc[2 * l]);

      yt = pSrc[2 * i + 1] - pSrc[2 * l + 1];
      pSrc[2 * i + 1] = (pSrc[2 * l + 1] + pSrc[2 * i + 1]);

      pSrc[2u * l] = xt;

      pSrc[2u * l + 1u] = yt;

    }                           // butterfly loop end

  }                             // groups loop end

  twidCoefModifier = twidCoefModifier << 1u;

}

static inline void applyWindow(q15_t *src, const q15_t *window, uint16_t len)
{
	while(len--){
		int32_t val = *src * *window++;
		*src++ = val >> 15;
	}
}

int ZeroFFT(q15_t *source, uint16_t length)
{
	uint16_t twidCoefModifier;
	uint16_t bitRevFactor;
	uint16_t *pBitRevTable;

	q15_t *pSrc = source;

	switch (length)
	  {

#if ZERO_FFT_MAX == 8192
	  case 4096u:
	    /*  Initializations of structure parameters for 4096 point FFT */

	    /*  Initialise the twiddle coef modifier value */
	    twidCoefModifier = 1u;
	    /*  Initialise the bit reversal table modifier */
	    bitRevFactor = 1u;
	    /*  Initialise the bit reversal table pointer */
	    pBitRevTable = (uint16_t *) armBitRevTable;

	    applyWindow(source, window_hanning_4096, 4096);

	    break;
#endif

#if ZERO_FFT_MAX >= 4096

	  case 2048u:
	    /*  Initializations of structure parameters for 2048 point FFT */

	    /*  Initialise the twiddle coef modifier value */
	    twidCoefModifier = 2u;
	    /*  Initialise the bit reversal table modifier */
	    bitRevFactor = 2u;
	    /*  Initialise the bit reversal table pointer */
	    pBitRevTable = (uint16_t *) & armBitRevTable[1];

	    applyWindow(source, window_hanning_2048, 2048);

	    break;

#endif

#if ZERO_FFT_MAX >= 2048
	  case 1024u:
	    /*  Initializations of structure parameters for 1024 point FFT */
	    twidCoefModifier = 4u;
	    bitRevFactor = 4u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[3];

	    applyWindow(source, window_hanning_1024, 1024);
	    break;
#endif

#if ZERO_FFT_MAX >= 1024
	  case 512u:
	    /*  Initializations of structure parameters for 512 point FFT */
	    twidCoefModifier = 8u;
	    bitRevFactor = 8u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[7];

	    applyWindow(source, window_hanning_512, 512);

	    break;
#endif

#if ZERO_FFT_MAX >= 512
	  case 256u:
	    /*  Initializations of structure parameters for 256 point FFT */
	    twidCoefModifier = 16u;
	    bitRevFactor = 16u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[15];

	    applyWindow(source, window_hanning_256, 256);

	    break;
#endif

	  case 128u:
	    /*  Initializations of structure parameters for 128 point FFT */
	    twidCoefModifier = 32u;
	    bitRevFactor = 32u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[31];

	    applyWindow(source, window_hanning_128, 128);

	    break;

	  case 64u:
	    /*  Initializations of structure parameters for 64 point FFT */
	    twidCoefModifier = 64u;
	    bitRevFactor = 64u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[63];

	    applyWindow(source, window_hanning_64, 64);

	    break;

	  case 32u:
	    /*  Initializations of structure parameters for 32 point FFT */
	    twidCoefModifier = 128u;
	    bitRevFactor = 128u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[127];

	    applyWindow(source, window_hanning_32, 32);

	    break;

	  case 16u:
	    /*  Initializations of structure parameters for 16 point FFT */
	    twidCoefModifier = 256u;
	    bitRevFactor = 256u;
	    pBitRevTable = (uint16_t *) & armBitRevTable[255];

	    applyWindow(source, window_hanning_16, 16);

	    break;

	  default:
	    /*  Reporting argument error if fftSize is not valid value */
		return -1;
	    break;
	  }

	//split the data
	q15_t *pOut = scratchData;
	for(int i=0; i<length; i++){
		*pOut++ = *pSrc++; //real
		*pOut++ = 0; //imaginary
	}

	arm_radix2_butterfly_q15(scratchData, length, (q15_t *)twiddleCoefQ15, twidCoefModifier);
	arm_bitreversal_q15(scratchData, length, bitRevFactor, pBitRevTable);

	pSrc = source;
	pOut = scratchData;
	for(int i=0; i<length; i++){
		  q15_t val = *pOut++;
		  uint32_t v = abs(val);
		  *pSrc++ = v;
		  pOut++; //discard imaginary phase val
	  }

	return 0;
}
