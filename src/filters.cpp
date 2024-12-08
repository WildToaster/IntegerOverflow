#include "filters.h"

namespace filters {

double lowPass(double input, double lastInput) {
    return (input + lastInput) / 2;
}

}