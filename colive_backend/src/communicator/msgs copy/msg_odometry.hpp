#pragma once

#include <eigen3/Eigen/Core>

// COVINS
#include "typedefs_base.hpp"

//SERIALIZATION
#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/types/concepts/pair_associative_container.hpp>
#include <cereal/types/base_class.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/access.hpp>
#include <cstdint>

namespace colive {

struct MsgOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using precision_t                   = TypeDefs::precision_t;
    using idpair                        = TypeDefs::idpair;

    using Vector3Type                   = TypeDefs::Vector3Type;
    using Matrix3Type                   = TypeDefs::Matrix3Type;
    using TransformType                 = TypeDefs::TransformType;

    using MsgTypeVector                 = TypeDefs::MsgTypeVector;

    struct compare_less{bool operator() (const MsgLandmark &a, const MsgLandmark &b) const;};

public:

    MsgOdometry();
    MsgOdometry(bool filesave);
    MsgOdometry(MsgTypeVector msgtype);

    // Interfaces
    auto SetMsgType(int msg_size)                                       ->void;
    auto SetMsgType(MsgTypeVector msgtype)                              ->void;

    // Infrastructure
    MsgTypeVector           msg_type                                                            = std::vector<uint32_t>(5);     // size, is_update, ID of Keyframe, ???,;
    bool                    is_update_msg                                                       = false;
    bool                    save_to_file                                                        = false;                        // indicates that this LM will be saved to a file, not send over network

    // Identifier
    idpair                  id;

    // Position
    Vector3Type             pos_ref;
    Vector3Type             pos_w;


protected:


};

} //end ns
