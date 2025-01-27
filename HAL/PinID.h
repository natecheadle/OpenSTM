#pragma once

namespace openstm::hal {

enum class PinID {
  Zero = 0b1,
  One = 0b10,
  Two = 0b100,
  Three = 0b1000,
  Four = 0b10000,
  Five = 0b100000,
  Six = 0b1000000,
  Seven = 0b10000000,
  Eight = 0b100000000,
  Nine = 0b1000000000,
  Ten = 0b10000000000,
  Eleven = 0b100000000000,
  Twelve = 0b1000000000000,
  Thirteen = 0b10000000000000,
  Fourteen = 0b100000000000000,
  Fifteen = 0b1000000000000000,
  All = 0xFFFF,
};
}
