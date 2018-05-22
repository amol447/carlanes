#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../helper.h"

TEST_CASE("rad2degTest"){
    REQUIRE(rad2deg(pi()/4) <= 45+0.0001);
    REQUIRE(rad2deg(pi()/4) >= 45 - 0.0001);
}