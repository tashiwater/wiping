/**
 * @file  SrdfParser.h
 * @brief SRDF parser class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

#ifndef __SRDF_PARSER_H__
#define __SRDF_PARSER_H__

/*----------------------------------------------------------------------
 Includes
 ----------------------------------------------------------------------*/
#include <string>
#include <vector>
#include <map>

/*----------------------------------------------------------------------
 Class Definitions
 ----------------------------------------------------------------------*/
namespace torobo_common
{

class SrdfParser
{
public:
    typedef struct
    {
        std::string groupStateName;
        std::string groupName;
        std::vector<std::string> jointNames;
        std::vector<double> positions;
    }GroupState_t;

    SrdfParser();
    virtual ~SrdfParser();

    std::map<std::string, SrdfParser::GroupState_t> Parse(const std::string srdfFileName);
    std::map<std::string, SrdfParser::GroupState_t> ParseText(const std::string srdfFileName);

protected:

private:
};

}

#endif
