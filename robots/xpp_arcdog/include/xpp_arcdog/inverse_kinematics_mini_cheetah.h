//
// Created by ray on 7/30/24.
//

#ifndef XPP_ARCDOG_INVERSE_KINEMATICS_ARCDOG_H
#define XPP_ARCDOG_INVERSE_KINEMATICS_ARCDOG_H

#include <xpp_vis/inverse_kinematics.h>
#include <xpp_arcdog/arcdogleg_inverse_kinematics.h>

namespace xpp
{

/**
 * @brief Inverse kinematics function for the cyberdog robot.
 */
    class InverseKinematicsMiniCheetah : public InverseKinematics
    {
    public:
        InverseKinematicsMiniCheetah() = default;
        virtual ~InverseKinematicsMiniCheetah() = default;

        /**
         * @brief Returns joint angles to reach for a specific foot position.
         * @param pos_B  3D-position of the foot expressed in the base frame (B).
         */
        Joints GetAllJointAngles(const EndeffectorsPos &pos_b) const override;

        /**
         * @brief Number of endeffectors (feet, hands) this implementation expects.
         */
        int GetEECount() const override { return 4; };

    private:
        Vector3d base2hip_LF_ = Vector3d(0.23536, 0.05, 0.0);
        ArcdoglegInverseKinematics leg;
    };

} /* namespace xpp */

#endif //XPP_ARCDOG_INVERSE_KINEMATICS_ARCDOG_H
