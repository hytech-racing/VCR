#include "gtest/gtest.h"
#include "IOExpanderUtils.h"

TEST (IOExpanderUtilsTest, getBit){
    bool[] expected = {{0, 1, 0, 1, 0, 1, 0, 1}, {0, 1, 0, 1, 0, 1, 0, 1}};
    for(int i=0; i<8; i++){
        ASSERT_EQ(expected[0][i], IOExpanderUtils::getBit(0, 0, i));
    }
    for(int i=0; i<8; i++){
        ASSERT_EQ(expected[1][i], IOExpanderUtils::getBit(0, 1, i));
    }
}
