// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_FUNCTIONS_DRIVE_H
#define HDDC2B_FUNCTIONS_DRIVE_H


#ifdef __cplusplus
extern "C" {
#endif


/**
 * Compute the force contribution of each of the two wheels to the drive's
 * attachment point.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dst An array with @p num_drv elements that represent the
 *                    (scalar) distance between each wheel and the central point
 *                    between the wheels, i.e. the wheels are @f$2 \cdot d_w@f$
 *                    apart from each other where @f$w_d@f$ represents the wheel
 *                    distance. The array is arranged as @f$
 *                    \begin{bmatrix}
 *                      d_{w,1} & \ldots & d_{w,n}
 *                    \end{bmatrix}@f$.
 * @param[in] cstr_off The castor offset, an array with @p num_drv elements that
 *                     represent the (scalar) distance between the wheel axle
 *                     and the drive's attachment point (to the platform). The
 *                     array is arranged as @f$
 *                     \begin{bmatrix}
 *                       co_1 & \ldots & co_n
 *                     \end{bmatrix}@f$.
 * @param[in] f_whl A matrix with two rows and @p num_drv columns where the rows
 *                  contain the linear force at the right and left wheel-ground
 *                  contact point, respectively. The matrix is arranged as @f[
 *                  \begin{bmatrix}
 *                    f_{1,r} & \ldots & f_{n,r} \\
 *                    f_{1,l} & \ldots & f_{n,l}
 *                  \end{bmatrix}
 *                   @f] and must be provided in column-major order.
 * @param[out] f_drv A matrix with two rows and @p num_drv columns where the
 *                   rows represent the linear force components at the drive's
 *                   attachment point (to the platform) in the longitudinal and
 *                   transverse direction, respectively. The forces' coordinates
 *                   are expressed in the individual drives' pivot frames. The
 *                   matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void hddc2b_drv_frc_gnd_to_pvt(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *f_whl,
        double *f_drv);


/**
 * Compute the force for each of the wheels in a wheel unit given the overall
 * force at an attachment point.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dst An array with @p num_drv elements that represent the
 *                    (scalar) distance between each wheel and the central point
 *                    between the wheels, i.e. the wheels are @f$2 \cdot d_w@f$
 *                    apart from each other where @f$w_d@f$ represents the wheel
 *                    distance. The array is arranged as @f$
 *                    \begin{bmatrix}
 *                      d_{w,1} & \ldots & d_{w,n}
 *                    \end{bmatrix}@f$.
 * @param[in] cstr_off The castor offset, an array with @p num_drv elements that
 *                     represent the (scalar) distance between the wheel axle
 *                     and the drive's attachment point (to the platform). The
 *                     array is arranged as @f$
 *                     \begin{bmatrix}
 *                       co_1 & \ldots & co_n
 *                     \end{bmatrix}@f$.
 * @param[in] f_drv A matrix with two rows and @p num_drv columns where the rows
 *                  represent the linear force components at the drive's
 *                  attachment point (to the platform) in the longitudinal and
 *                  transverse direction, respectively. The forces' coordinates
 *                  are expressed in the individual drives' pivot frames. The
 *                  matrix is arranged as @f[
 *                  \begin{bmatrix}
 *                    f_{1,x} & \ldots & f_{n,x} \\
 *                    f_{1,y} & \ldots & f_{n,y}
 *                  \end{bmatrix}
 *                  @f] and must be provided in column-major order.
 * @param[out] f_whl A matrix with two rows and @p num_drv columns where the
 *                   rows contain the linear force at the right and left
 *                   wheel-ground contact point, respectively. The matrix is
 *                   arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,r} & \ldots & f_{n,r} \\
 *                     f_{1,l} & \ldots & f_{n,l}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void hddc2b_drv_frc_pvt_to_gnd(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *f_drv,
        double *f_whl);


/**
 * Compute the velocity of the drive's attachment point due to the two wheels'
 * velocities.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dst An array with @p num_drv elements that represent the
 *                    (scalar) distance between each wheel and the central point
 *                    between the wheels, i.e. the wheels are @f$2 \cdot d_w@f$
 *                    apart from each other where @f$w_d@f$ represents the wheel
 *                    distance. The array is arranged as @f$
 *                    \begin{bmatrix}
 *                      d_{w,1} & \ldots & d_{w,n}
 *                    \end{bmatrix}@f$.
 * @param[in] cstr_off The castor offset, an array with @p num_drv elements that
 *                     represent the (scalar) distance between the wheel axle
 *                     and the drive's attachment point (to the platform). The
 *                     array is arranged as @f$
 *                     \begin{bmatrix}
 *                       co_1 & \ldots & co_n
 *                     \end{bmatrix}@f$.
 * @param[in] xd_whl A matrix with two rows and @p num_drv columns where the
 *                   rows contain the linear velocity at the right and left
 *                   wheel-ground contact point, respectively. The matrix is
 *                   arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,r} & \ldots & \dot{X}_{n,r} \\
 *                     \dot{X}_{1,l} & \ldots & \dot{X}_{n,l}
 *                   \end{bmatrix}
 *                    @f] and must be provided in column-major order.
 * @param[out] xd_drv A matrix with two rows and @p num_drv columns where the
 *                    rows represent the linear velocity components of the
 *                    drive's attachment point (to the platform) in the
 *                    longitudinal and transverse direction, respectively. The
 *                    linear velocities' reference point is the origin of the
 *                    respective pivot frames. Their coordinates are expressed
 *                    in these pivot frames. The matrix is arranged as @f[
 *                    \begin{bmatrix}
 *                      \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                      \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                    \end{bmatrix}
 *                    @f] and will be provided in column-major order.
 */
void hddc2b_drv_vel_gnd_to_pvt(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *xd_whl,
        double *xd_drv);


/**
 * Compute the velocity at both wheel-ground contact points from the velocity
 * of the drive's attachment point, i.e. the inverse of
 * @ref hddc2b_drv_vel_gnd_to_pvt. All arguments are arranged as there.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] whl_dst The distance between the two wheels of each drive.
 * @param[in] cstr_off The castor offset of each drive.
 * @param[in] xd_drv The longitudinal and transverse velocity of each drive's
 *                   attachment point, expressed in its pivot frame.
 * @param[out] xd_whl The linear velocity at the right and left wheel-ground
 *                    contact points.
 */
void hddc2b_drv_vel_pvt_to_gnd(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *xd_drv,
        double *xd_whl);


/**
 * Compute the pivot alignment distance of each drive, i.e. the angle
 * @f$\operatorname{atan2}(\dot{X}_{i,y}, \dot{X}_{i,x})@f$ between the drive's
 * rolling direction and the velocity of its attachment point.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] xd_drv The longitudinal and transverse velocity of each drive's
 *                   attachment point, arranged as in
 *                   @ref hddc2b_drv_vel_gnd_to_pvt.
 * @param[out] dst The alignment distance of each drive.
 * @param[in] inc_dst Increment of the @p dst array.
 */
void hddc2b_drv_vel_algn_dst(
        int num_drv,
        const double *xd_drv,
        double *dst,
        int inc_dst);


/**
 * Compute each drive's scrub angle under a platform twist, i.e. the angle
 * between its rolling direction and the axle centre velocity of a castor that
 * turns with the platform:
 * @f[
 *   dst_i = \operatorname{atan2}(\dot{X}_{i,y} - l_i \omega_p, \dot{X}_{i,x})
 * @f]
 * A drive that rolls backwards gets @f$+\pi@f$; a drive at rest gets zero.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] cstr_off The castor offset @f$l_i@f$ of each drive.
 * @param[in] omega_pltf The platform's angular velocity @f$\omega_p@f$.
 * @param[in] xd_drv The pivot velocities, arranged as in
 *                   @ref hddc2b_drv_vel_gnd_to_pvt.
 * @param[out] dst The scrub angle of each drive.
 * @param[in] inc_dst Increment of the @p dst array.
 * @param[out] spd The speed @f$v_i@f$ of each drive's axle centre.
 * @param[in] inc_spd Increment of the @p spd array.
 */
void hddc2b_drv_vel_scrb_dst(
        int num_drv,
        const double *cstr_off,
        double omega_pltf,
        const double *xd_drv,
        double *dst,
        int inc_dst,
        double *spd,
        int inc_spd);


/**
 * Compute the transverse pivot velocity that removes each drive's scrub angle:
 * @f[
 *   \bar{\dot{X}}_{i,y} = l_i \left(
 *     \operatorname{clip}\left(
 *       \min\left(\frac{1}{\tau}, \frac{v_i}{l_i}\right) dst_i,
 *       \pm\dot{q}_{max}\right)
 *     + \omega_p \right)
 * @f]
 * A castor aligns by rolling, so it is never asked to turn faster than its
 * own time constant @f$l_i / v_i@f$; a drive at rest is not asked to turn.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] cstr_off The castor offset @f$l_i@f$ of each drive.
 * @param[in] tau The shortest alignment time constant; must be positive.
 * @param[in] qd_max The maximum castor rate relative to the platform; must be
 *                   non-negative.
 * @param[in] omega_pltf The platform's angular velocity @f$\omega_p@f$.
 * @param[in] dst The scrub angles from @ref hddc2b_drv_vel_scrb_dst.
 * @param[in] inc_dst Increment of the @p dst array.
 * @param[in] spd The axle centre speeds from @ref hddc2b_drv_vel_scrb_dst.
 * @param[in] inc_spd Increment of the @p spd array.
 * @param[out] xd_ref The transverse velocity reference of each pivot.
 * @param[in] inc_ref Increment of the @p xd_ref array; two writes the
 *                    transverse row of a pivot velocity matrix.
 */
void hddc2b_drv_vel_algn_ref(
        int num_drv,
        const double *cstr_off,
        double tau,
        double qd_max,
        double omega_pltf,
        const double *dst,
        int inc_dst,
        const double *spd,
        int inc_spd,
        double *xd_ref,
        int inc_ref);


#ifdef __cplusplus
}
#endif

#endif
