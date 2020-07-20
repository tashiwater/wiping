#include <gtest/gtest.h>
#include <ros/package.h>
#include <torobo_common/SrdfParser.h>

using namespace std;

class SrdfParserTestFixture : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
    }
    virtual void TearDown()
    {
    }
};

TEST_F(SrdfParserTestFixture, ParseSrdf)
{
    std::string packagePath = ros::package::getPath("torobo_common");
    std::string srdfPath = packagePath + "/resources/torobo.srdf";

    torobo_common::SrdfParser parser;
    map<string, torobo_common::SrdfParser::GroupState_t> state = parser.Parse(srdfPath);

    ASSERT_TRUE(state.count("right_arm") > 0);
    ASSERT_EQ("right_arm/joint_1", state["right_arm"].jointNames[0]);
    ASSERT_EQ("right_arm/joint_2", state["right_arm"].jointNames[1]);
    ASSERT_EQ("right_arm/joint_3", state["right_arm"].jointNames[2]);
    ASSERT_EQ("right_arm/joint_4", state["right_arm"].jointNames[3]);
    ASSERT_EQ("right_arm/joint_5", state["right_arm"].jointNames[4]);
    ASSERT_EQ("right_arm/joint_6", state["right_arm"].jointNames[5]);
    ASSERT_DOUBLE_EQ(0.0,    state["right_arm"].positions[0]);
    ASSERT_DOUBLE_EQ(1.5708, state["right_arm"].positions[1]);
    ASSERT_DOUBLE_EQ(0.0,    state["right_arm"].positions[2]);
    ASSERT_DOUBLE_EQ(0.0,    state["right_arm"].positions[3]);
    ASSERT_DOUBLE_EQ(0.0,    state["right_arm"].positions[4]);
    ASSERT_DOUBLE_EQ(0.0,    state["right_arm"].positions[5]);

    ASSERT_TRUE(state.count("left_arm") > 0);
    ASSERT_EQ("left_arm/joint_1", state["left_arm"].jointNames[0]);
    ASSERT_EQ("left_arm/joint_2", state["left_arm"].jointNames[1]);
    ASSERT_EQ("left_arm/joint_3", state["left_arm"].jointNames[2]);
    ASSERT_EQ("left_arm/joint_4", state["left_arm"].jointNames[3]);
    ASSERT_EQ("left_arm/joint_5", state["left_arm"].jointNames[4]);
    ASSERT_EQ("left_arm/joint_6", state["left_arm"].jointNames[5]);
    ASSERT_DOUBLE_EQ(0.0,    state["left_arm"].positions[0]);
    ASSERT_DOUBLE_EQ(1.5708, state["left_arm"].positions[1]);
    ASSERT_DOUBLE_EQ(0.0,    state["left_arm"].positions[2]);
    ASSERT_DOUBLE_EQ(0.0,    state["left_arm"].positions[3]);
    ASSERT_DOUBLE_EQ(0.0,    state["left_arm"].positions[4]);
    ASSERT_DOUBLE_EQ(0.0,    state["left_arm"].positions[5]);

    ASSERT_TRUE(state.count("torso_head") > 0);
    ASSERT_EQ("torso_head/joint_1", state["torso_head"].jointNames[0]);
    ASSERT_EQ("torso_head/joint_2", state["torso_head"].jointNames[1]);
    ASSERT_EQ("torso_head/joint_3", state["torso_head"].jointNames[2]);
    ASSERT_EQ("torso_head/joint_4", state["torso_head"].jointNames[3]);
    ASSERT_DOUBLE_EQ(0.0,    state["torso_head"].positions[0]);
    ASSERT_DOUBLE_EQ(0.0,    state["torso_head"].positions[1]);
    ASSERT_DOUBLE_EQ(0.0,    state["torso_head"].positions[2]);
    ASSERT_DOUBLE_EQ(0.0,    state["torso_head"].positions[3]);
}
