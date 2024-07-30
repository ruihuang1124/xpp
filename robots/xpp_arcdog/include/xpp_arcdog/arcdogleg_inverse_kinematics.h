/******************************************************************************
Copyright (c) 2017, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef XPP_ARCDOG_ARCDOGLEG_INVERSE_KINEMATICS_H
#define XPP_ARCDOG_ARCDOGLEG_INVERSE_KINEMATICS_H


#include <Eigen/Dense>

namespace xpp {

    enum ArcdogJointID
    {
        HAA = 0, HFE, KFE, ArcdoglegJointCount
    };

/**
 * @brief Converts a arcdog foot position to joint angles.
 */
    class ArcdoglegInverseKinematics
    {
    public:
        using Vector3d = Eigen::Vector3d;
        enum KneeBend
        {
            Forward, Backward
        };

        /**
         * @brief Default c'tor initializing leg lengths with standard values.
         */
        ArcdoglegInverseKinematics() = default;
        virtual ~ArcdoglegInverseKinematics() = default;

        /**
         * @brief Returns the joint angles to reach a Cartesian foot position.
         * @param ee_pos_H  hip-aa (H) 坐标系中脚的位置 xyz
         */
        Vector3d GetJointAngles(const Vector3d &ee_pos_H, int _sideSign) const;

        /**
         * @brief Restricts the joint angles to lie inside the feasible range
         * @param q[in/out]  Current joint angle that is adapted if it exceeds
         * the specified range.
         * @param joint  Which joint (HAA, HFE, KFE) this value represents.
         */
        void EnforceLimits(double &q, ArcdogJointID joint) const;

    private:
        double _abadLinkLength = 0.10715;
        double _hipLinkLength = 0.2;
        double _kneeLinkLength = 0.217;
        static double q1_ik(double py, double pz, double l1);
        static double q3_ik(double b3z, double b4z, double b);
        static double q2_ik(double q1, double q3, double px, double py, double pz, double b3z, double b4z);
    };

} /* namespace xpp */

#endif //XPP_ARCDOG_ARCDOGLEG_INVERSE_KINEMATICS_H
