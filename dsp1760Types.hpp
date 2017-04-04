#ifndef dsp1760_TYPES_HPP
#define dsp1760_TYPES_HPP

#include <base/Time.hpp>

namespace dsp1760
{
  namespace samples
  {
    struct Bias
    {
      base::Time time;
      double bias_value;
    };
  }
}

#endif
