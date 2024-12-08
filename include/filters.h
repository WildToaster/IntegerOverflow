#pragma once

namespace filters {

// Precents the input from changing too quickly
// See https://dobrian.github.io/cmp/topics/filters/lowpassfilter.html
extern double lowPass(double input, double lastInput);

}