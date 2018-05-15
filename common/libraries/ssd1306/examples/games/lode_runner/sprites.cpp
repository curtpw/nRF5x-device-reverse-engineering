/*
    MIT License

    Copyright (c) 2018, Alexey Dynda

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include "sprites.h"

PROGMEM const uint8_t bgSprites[5][8] =
{
    {
        0B01110111,
        0B01110111,
        0B01110000,
        0B01110111,
        0B01110111,
        0B00000111,
        0B01110111,
        0B01110111,
    },
    {
        0B00010001,
        0B11111111,
        0B00010001,
        0B00010001,
        0B00010001,
        0B11111111,
        0B00010001,
        0B00000000,
    },
    {
        0B00000010,
        0B00000010,
        0B00000010,
        0B00000010,
        0B00000010,
        0B00000010,
        0B00000010,
        0B00000010,
    },
    {
        0B00000000,
        0B11000000,
        0B11100000,
        0B10110000,
        0B11010000,
        0B10100000,
        0B11000000,
        0B00000000,
    },
    {
        0B01111111,
        0B01111111,
        0B01111111,
        0B01110111,
        0B01111111,
        0B01111111,
        0B01111111,
        0B00000000,
    },
};

const PROGMEM uint8_t playerFlyingImage[MAN_ANIM_MAX][2][8] = {
{
    {
        0B00000000,
        0B00001000,
        0B10010000,
        0B01110100,
        0B00111110,
        0B01110100,
        0B10010000,
        0B00001000,
    },
    {
        0B00000000,
        0B00100000,
        0B10010000,
        0B01110100,
        0B00111110,
        0B01110100,
        0B10010000,
        0B00100000,
    },
},
{
    {
        0B00100000,
        0B10010000,
        0B01110100,
        0B00111110,
        0B01110100,
        0B01010000,
        0B01001000,
        0B00000000,
    },
    {
        0B01001000,
        0B01010000,
        0B01110100,
        0B00111110,
        0B01110100,
        0B10010000,
        0B00100000,
        0B00000000,
    },
},
{
    {
        0B00000000,
        0B00100000,
        0B00010000,
        0B11110100,
        0B00111110,
        0B01110100,
        0B10010000,
        0B00100000,
    },
    {
        0B00000000,
        0B00100000,
        0B10010000,
        0B01110100,
        0B00111110,
        0B11110100,
        0B00100000,
        0B00000000,
    },
},
{
    {
        0B00000000,
        0B00100000,
        0B00010000,
        0B11110100,
        0B00111110,
        0B01110100,
        0B10010000,
        0B00100000,
    },
    {
        0B00000000,
        0B00100000,
        0B10010000,
        0B01110100,
        0B00111110,
        0B11110100,
        0B00100000,
        0B00000000,
    },
},
{
    {
        0B00000110,
        0B00001000,
        0B00111110,
        0B00110000,
        0B00111000,
        0B00110110,
        0B00111000,
        0B00111000,
    },
    {
        0B00000000,
        0B00000110,
        0B00111000,
        0B00110000,
        0B00111110,
        0B00110000,
        0B00111000,
        0B00111000,
    },
},
{
    {
        0B00111000,
        0B00111000,
        0B00110110,
        0B00111000,
        0B00110000,
        0B00111110,
        0B00001000,
        0B00000110,
    },
    {
        0B00111000,
        0B00111000,
        0B00110000,
        0B00111110,
        0B00110000,
        0B00111000,
        0B00000110,
        0B00000000,
    },
},
};

const PROGMEM uint8_t coinImage[8] =
{
    0B00000000,
    0B00000000,
    0B00111100,
    0B01100110,
    0B00111100,
    0B00000000,
    0B00000000,
    0B00000000,
};
