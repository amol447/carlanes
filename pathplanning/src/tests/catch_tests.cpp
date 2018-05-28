#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "../helper.h"
#include <time.h>
TEST_CASE("rad2degTest"){
    REQUIRE(rad2deg(pi()/4) <= 45+0.0001);
    REQUIRE(rad2deg(pi()/4) >= 45 - 0.0001);
}

TEST_CASE("rotation_translation_inverse_test"){
    srand( (unsigned) time(NULL));
    bool flag=true;
    for(unsigned int i=0;i<10;i++) {
        CartesianPoint start(rand()*30.0/RAND_MAX, rand()*30.0/RAND_MAX);
        CartesianPoint ref(rand()*30.0/RAND_MAX, rand()*30.0/RAND_MAX);
        AngleInRadians ref_angle(2*pi()*rand()/RAND_MAX);
        CartesianPoint back_n_forth = inverse_rotation_translation(rotation_translation(start, ref, ref_angle), ref,
                                                                   ref_angle);
        if( (fabs(back_n_forth.x-start.x) > 0.0001) or (fabs(back_n_forth.y-start.y) > 0.0001))
            flag= false;
    }
    REQUIRE(flag);
}

TEST_CASE("rotation_translation_test"){
    CartesianPoint start(2.0,3.0);
    CartesianPoint ref(1.0,2.0);
    AngleInRadians ref_angle(deg2rad(45));
    auto end = rotation_translation(start,ref,ref_angle);
    REQUIRE(abs(end.x-3.0/sqrt(2.0)) < 0.0001);
    REQUIRE(abs(end.y+1.0/sqrt(2.0))< 0.0001);
}

TEST_CASE("moveForwardTest"){
REQUIRE(true);
}