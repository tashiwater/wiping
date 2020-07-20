/**
 * @file  ToroboRbdlModel.h
 * @brief Torobo Rigid Body Dynamics Library class
 *
 * @par   Copyright © 2018 Tokyo Robotics Inc. All rights reserved.
 */

 /*----------------------------------------------------------------------
  Includes
  ----------------------------------------------------------------------*/
#include "ToroboRbdlModel.h"

#ifndef RBDL_BUILD_ADDON_URDFREADER
    #error "Error: RBDL addon URDFReader not enabled."
#endif

#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


namespace torobo
{

/*----------------------------------------------------------------------
 Privat Global Variables
 ----------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 Public Method Definitions
 ----------------------------------------------------------------------*/
ToroboRbdlModel::ToroboRbdlModel()
{
    rbdl_check_api_version (RBDL_API_VERSION);

    model_.reset(new RigidBodyDynamics::Model());
    if(model_ == nullptr)
    {
        return;
    }

    if (!LoadModelFromUrdfRosParam("/torobo/robot_description"))
    {
        ROS_ERROR("Loading Failed.");
        return;
    }
    ROS_INFO("Loading Succeeded.");

    InitBodyNameMap();

    model_->gravity = Vector3d (0., 0., -9.80665);

    Q_ = VectorNd::Zero (model_->dof_count);
    QDot_ = VectorNd::Zero (model_->dof_count);
    QDDot_ = VectorNd::Zero (model_->dof_count);
    Tau_ = VectorNd::Zero (model_->dof_count);
    H_ = MatrixNd::Zero(model_->dof_count, model_->dof_count);
}

ToroboRbdlModel::~ToroboRbdlModel()
{
}

void ToroboRbdlModel::Print()
{
    // std::cout << "Gravity: " << model_->gravity.transpose() << std::endl;
    // std::cout << "Q      : " << Q_.transpose() << std::endl;
    // std::cout << "QDot   : " << QDot_.transpose() << std::endl;
    // std::cout << "QDDot  : " << QDDot_.transpose() << std::endl;
    // std::cout << "Tau    : " << Tau_.transpose() << std::endl;
    // std::cout << "H      : " << std::endl;
    // std::cout << H_ << std::endl;
    for(int row = 0; row < H_.rows();row++)
    {
        printf("%d|",row);
        for(int col = 0;col < H_.cols();col++)
        {
            if(col==row)
            {
                printf("%20.20lf ",H_(row,col));
            }
        }
        printf("\n");
    }
}

int ToroboRbdlModel::GetBodyId(const std::string& joint_name)
{
    if(bodyNameIdMap_.count(joint_name) == 0)
    {
        return -1;
    }
    int id = bodyNameIdMap_[joint_name];
    if(id >= model_->dof_count)
    {
        return -1;
    }
    return id;
}
int ToroboRbdlModel::GetJointDoF()
{
    return model_->dof_count;
}
void ToroboRbdlModel::UpdateQ(const std::vector<std::string>& joint_names, std::vector<double> Q)
{
    if(joint_names.size() != Q.size())
    {
        return;
    }
    for(int i = 0; i < joint_names.size(); i++)
    {
        string name = joint_names[i];
        int id = GetBodyId(name);
        if(id < 0)
        {
            continue;
        }
        Q_[id] = Q[i];
    }
}

void ToroboRbdlModel::UpdateQDot(const std::vector<std::string>& joint_names, std::vector<double> QDot)
{
    if(joint_names.size() != QDot.size())
    {
        return;
    }
    for(int i = 0; i < joint_names.size(); i++)
    {
        string name = joint_names[i];
        int id = GetBodyId(name);
        if(id < 0)
        {
            continue;
        }
        QDot_[id] = QDot[i];
    }
}
void ToroboRbdlModel::UpdateQDDot(const std::vector<std::string>& joint_names, std::vector<double> QDDot)
{
    if(joint_names.size() != QDDot.size())
    {
        return;
    }
    for(int i = 0; i < joint_names.size(); i++)
    {
        string name = joint_names[i];
        int id = GetBodyId(name);
        if(id < 0)
        {
            continue;
        }
        QDDot_[id] = QDDot[i];
    }
}
void ToroboRbdlModel::UpdateTau(const std::vector<std::string>& joint_names, std::vector<double> Tau)
{
    if(joint_names.size() != Tau.size())
    {
        return;
    }
    for(int i = 0; i < joint_names.size(); i++)
    {
        string name = joint_names[i];
        int id = GetBodyId(name);
        if(id < 0)
        {
            continue;
        }
        Tau_[id] = Tau[i];
    }
}

void ToroboRbdlModel::CalcForwardDynamics()
{
    ForwardDynamics (*model_, Q_, QDot_, Tau_, QDDot_);
}

void ToroboRbdlModel::CalcInverseDynamics()
{
    InverseDynamics (*model_, Q_, QDot_, QDDot_, Tau_, NULL);
}

void ToroboRbdlModel::CalcCompositeRigidBodyAlgorithm()
{
    CompositeRigidBodyAlgorithm(*model_, Q_, H_);
}

std::vector<double> ToroboRbdlModel::GetTau(const std::string& prefix, int jointsNum)
{
    vector<double> tau;
    for(int i = 0; i < jointsNum; i++)
    {
        string name = prefix + "/joint_" + to_string(i+1);
        int id = GetBodyId(name);
        tau.push_back(Tau_[id]);
    }
    return tau;
}

std::vector<double> ToroboRbdlModel::GetInertiaDiagonal(const std::string& prefix, int jointsNum)
{
    vector<double> diagonalElem;
    for(int i = 0; i < jointsNum; i++)
    {
        string name = prefix + "/joint_" + to_string(i+1);
        int id = GetBodyId(name);
        diagonalElem.push_back(H_(id,id));
    }
    return diagonalElem;
}
unsigned int ToroboRbdlModel::AddFixedBody(const std::string& prefix,int jointsNum,double mass,const RigidBodyDynamics::Math::Vector3d& com,const RigidBodyDynamics::Math::Matrix3d& InertiaMat)
{
    string  addedLinkName = prefix + "/added_link";
    const Joint joint = Joint(JointTypeFixed);
    unsigned int addedLinkId = 0;

    string name;
    if(!GetLastLinkName(name, prefix, jointsNum))
    {
        ROS_ERROR("Failed to AddFixedBody");
        return -1;
    }
    unsigned int parentId = model_->GetBodyId(name.c_str());

    const Body newBody = Body(mass , com, InertiaMat);
    SpatialTransform sp;
    if(parentBodyMap_.count(prefix) == 0)
    {
        // parentの状態を保存しておく
        if(model_->IsFixedBodyId(parentId))
        {
            FixedBody& fixed_parent_body = model_->mFixedBodies[parentId - model_->fixed_body_discriminator];
            unsigned int moveableParentId = fixed_parent_body.mMovableParent;
            parentBodyMap_[prefix] = model_->mBodies[moveableParentId];
        }
        else
        {
            Body& parent_body = model_->mBodies[parentId];
            parentBodyMap_[prefix] = parent_body;
        }

        // AddBodyを行う
        int addedBodyIndex = model_->mFixedBodies.size();
        model_->AddBody(parentId, sp, joint,newBody,addedLinkName);
        addedBodyIndexMap_[prefix] = addedBodyIndex;
    }
    else
    {
        // parentを一度元の状態に戻す
        unsigned int moveableParent = parentId;
        if(model_->IsFixedBodyId(parentId))
        {
            FixedBody& fixed_parent_body = model_->mFixedBodies[parentId - model_->fixed_body_discriminator];
            moveableParent = fixed_parent_body.mMovableParent;
        }
        Body& parent_body = parentBodyMap_[prefix];
        model_->mBodies[moveableParent] = parent_body;
        model_->I[moveableParent] = SpatialRigidBodyInertia::createFromMassComInertiaC ( 
            parent_body.mMass, 
            parent_body.mCenterOfMass, 
            parent_body.mInertia);

        // 追加済みのFixedBodyを一度削除する
        int addedBodyIndex = addedBodyIndexMap_[prefix];
        model_->mFixedBodies.erase(model_->mFixedBodies.begin() + addedBodyIndex);
        model_->mBodyNameMap.erase(addedLinkName);

        // 再度AddBodyする
        addedBodyIndex = model_->mFixedBodies.size();
        model_->AddBody(parentId, sp, joint, newBody, addedLinkName);
        addedBodyIndexMap_[prefix] = addedBodyIndex;
    }

    return parentId;
}

int ToroboRbdlModel::ShowBody()
{
    for(int i = 0 ; i < 15;i++)
    {
        cout << "------------------------------------------" << endl;
        cout << i << endl;
        cout << model_->GetBodyName(i) << endl;
        cout << model_->GetJointFrame(i) << endl;
    }
}
/*----------------------------------------------------------------------
 Protected Method Definitions
 ----------------------------------------------------------------------*/
bool ToroboRbdlModel::LoadModelFromUrdfFile(std::string urdfFilePath)
{
    ROS_INFO("Loading model from URDF file: [%s]", urdfFilePath.c_str());
    if (!Addons::URDFReadFromFile (urdfFilePath.c_str(), model_.get(), false))
    {
        return false;
    }
    return true;
}

bool ToroboRbdlModel::LoadModelFromUrdfRosParam(std::string urdfParamName)
{
    ros::NodeHandle node;
    ROS_INFO("Loading model from URDF rosparam: [%s]", urdfParamName.c_str());
    if(!node.hasParam(urdfParamName))
    {
        ROS_ERROR("rosparam name: [%s] is not found.", urdfParamName.c_str());
        return false;
    }

    string xml_string;
    node.getParam(urdfParamName, xml_string);
    if (!Addons::URDFReadFromString (xml_string.c_str(), model_.get(), false))
    {
        return false;
    }
    return true;
}

std::string replaceString(const std::string target,
                          const std::string from,
                          const std::string to)
{
  std::string result = target;
  std::string::size_type pos = 0;
  while(pos = result.find(from, pos), pos != std::string::npos) {
    result.replace(pos, from.length(), to);
    pos += to.length();
  }
  return result;
}

void ToroboRbdlModel::InitBodyNameMap()
{
    bodyNameIdMap_.clear();
    bodyIdNameMap_.clear();
    for(auto it = model_->mBodyNameMap.begin(); it != model_->mBodyNameMap.end(); ++it)
    {
        string name = it->first;
        int id = it->second - 1;
        // ignore root and not movable joint
        if(id < 0 || id >= model_->dof_count)
        {
            continue;
        }
        name = replaceString(name, "link", "joint");
        bodyNameIdMap_.insert(make_pair(name, id));
        bodyIdNameMap_.insert(make_pair(id, name));
        // std::cout << name << ", " << id << std::endl;
    }
}

bool ToroboRbdlModel::GetLastLinkName(std::string& outLinkName, const std::string& prefix, const int jointsNum)
{
    int idx = jointsNum;
    std::string lastLinkName = "";
    while(true)
    {
        string name = prefix + "/link_" + to_string(idx);
        int bodyId = model_->GetBodyId(name.c_str());
        if(bodyId == std::numeric_limits<unsigned int>::max())
        {
            break;
        }
        lastLinkName = name;
        idx++;
    }
    if(lastLinkName == "")
    {
        return false;
    }
    outLinkName = lastLinkName;
    return true;
}

}
