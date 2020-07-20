/**
 * @file  SrdfParser.cpp
 * @brief SRDF parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include <iostream>
#include <torobo_common/SrdfParser.h>
#include <tinyxml2.h>

using namespace std;

namespace torobo_common
{
/*----------------------------------------------------------------------
 Private Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
SrdfParser::SrdfParser()
{
}

SrdfParser::~SrdfParser()
{
}

std::map<std::string, SrdfParser::GroupState_t> SrdfParser::Parse(const std::string srdfFileName)
{
    tinyxml2::XMLDocument doc;
    doc.LoadFile(srdfFileName.c_str());

    tinyxml2::XMLElement * root = doc.RootElement();
    tinyxml2::XMLElement * elem = doc.FirstChildElement("robot")->FirstChildElement("group_state");

    map<string, GroupState_t> groupStateMap;
    while(elem)
    {
        GroupState_t state;

        state.groupStateName = elem->Attribute("name");
        state.groupName = elem->Attribute("group");

        tinyxml2::XMLElement* joint = elem->FirstChildElement("joint");
        while(joint)
        {
            string jointName = joint->Attribute("name");
            string val = joint->Attribute("value");
            state.jointNames.push_back(jointName);
            state.positions.push_back(stod(val));
            joint = joint->NextSiblingElement("joint");
        }
        groupStateMap.insert(make_pair(state.groupName, state));
        elem = elem->NextSiblingElement("group_state");
    }
#if 0
    for(auto itr = groupStateMap.begin(); itr != groupStateMap.end(); ++itr)
    {
        cout << itr->first << ", " << itr->second.groupStateName << endl;
        for(int i = 0; i < itr->second.jointNames.size(); i++)
        {
            cout << itr->second.jointNames[i] << ", " << itr->second.positions[i] << endl;
        }
    }
#endif

    return groupStateMap;
}

std::map<std::string, SrdfParser::GroupState_t> SrdfParser::ParseText(const std::string srdfText)
{
    tinyxml2::XMLDocument doc;
    //doc.LoadFile(srdfFileName.c_str());
    doc.Parse(srdfText.c_str());

    tinyxml2::XMLElement * root = doc.RootElement();
    tinyxml2::XMLElement * elem = doc.FirstChildElement("robot")->FirstChildElement("group_state");

    map<string, GroupState_t> groupStateMap;
    while(elem)
    {
        GroupState_t state;

        state.groupStateName = elem->Attribute("name");
        state.groupName = elem->Attribute("group");

        tinyxml2::XMLElement* joint = elem->FirstChildElement("joint");
        while(joint)
        {
            string jointName = joint->Attribute("name");
            string val = joint->Attribute("value");
            state.jointNames.push_back(jointName);
            state.positions.push_back(stod(val));
            joint = joint->NextSiblingElement("joint");
        }
        groupStateMap.insert(make_pair(state.groupName, state));
        elem = elem->NextSiblingElement("group_state");
    }
#if 0
    for(auto itr = groupStateMap.begin(); itr != groupStateMap.end(); ++itr)
    {
        cout << itr->first << ", " << itr->second.groupStateName << endl;
        for(int i = 0; i < itr->second.jointNames.size(); i++)
        {
            cout << itr->second.jointNames[i] << ", " << itr->second.positions[i] << endl;
        }
    }
#endif

    return groupStateMap;
}

/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/

}
