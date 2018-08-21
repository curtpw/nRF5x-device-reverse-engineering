# FIR Filter Arduino Library

[![Say Thanks](https://img.shields.io/badge/Say%20Thanks-!-1EAEDB.svg)](https://saythanks.io/to/leemangeo)

A flexible FIR filter for the Arduino or other CPP micro. This was inspired by
[Sebastian Nilsson's](https://github.com/sebnil)
[FIR library](https://github.com/sebnil/FIR-filter-Arduino-Library)
, but is a more generalized implementation that can be used with multiple data
types. There is also an extensive example gallery for those not familiar with
class templates or FIR filtering.

This library does not calculate the filter coefficients for you, but the online
[T-Filter](http://t-filter.engineerjs.com) tool does an excellent job.

## Installation
Checkout the [Arduino library installation page](https://www.arduino.cc/en/Guide/Libraries) for details on how to install this library.

## Examples
* **[determining_gain](examples/determining_gain)** - Demonstrates how to
  determine the appropriate gain for the filter.
* **[moving_average](examples/moving_average)** - The FIR filter can be used as
  an efficient moving average filter.
* **[multiple_filters](examples/multiple_filters)** - Shows how to construct and
  use multiples filters in a single sketch.

## License
This software is licensed under the [MIT license](LICENSE). If you find the
code useful, feel free to drop a note and
[say thanks](https://saythanks.io/to/leemangeo) or consider using
[Leeman Geophysical LLC](https://www.leemangeophysical.com) for your next
project!
