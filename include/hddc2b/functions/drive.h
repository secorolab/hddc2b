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
 * Compute the velocity at each of the two wheel-ground contact points given the
 * velocity of the drive's attachment point. This is the kinematic dual of the
 * (force) mapping @ref hddc2b_drv_frc_gnd_to_pvt (i.e. its transpose) and the
 * inverse of @ref hddc2b_drv_vel_gnd_to_pvt.
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
 * @param[in] xd_drv A matrix with two rows and @p num_drv columns where the
 *                   rows represent the linear velocity components of the
 *                   drive's attachment point (to the platform) in the
 *                   longitudinal and transverse direction, respectively. The
 *                   linear velocities' reference point is the origin of the
 *                   respective pivot frames. Their coordinates are expressed
 *                   in these pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and must be provided in column-major order.
 * @param[out] xd_whl A matrix with two rows and @p num_drv columns where the
 *                    rows contain the linear velocity at the right and left
 *                    wheel-ground contact point, respectively. The matrix is
 *                    arranged as @f[
 *                    \begin{bmatrix}
 *                      \dot{X}_{1,r} & \ldots & \dot{X}_{n,r} \\
 *                      \dot{X}_{1,l} & \ldots & \dot{X}_{n,l}
 *                    \end{bmatrix}
 *                     @f] and will be provided in column-major order.
 */
void hddc2b_drv_vel_pvt_to_gnd(
        int num_drv,
        const double *whl_dst,
        const double *cstr_off,
        const double *xd_drv,
        double *xd_whl);


/**
 * Compute a signed pivot alignment distance for each drive given its commanded
 * attachment velocity. The distance is the angle between the drive's rolling
 * direction (the pivot frame's x-axis) and the commanded velocity, i.e.
 * @f$dst_i = \operatorname{atan2}(\dot{X}_{i,y}, \dot{X}_{i,x})@f$. Rotating the
 * pivot by this angle lets the drive roll without lateral scrubbing.
 *
 * This is the velocity-domain counterpart of @ref hddc2b_pltf_drv_algn_dst:
 * because velocity distribution (@ref hddc2b_pltf_vel_pltf_to_pvt) is a unique
 * map, each drive's desired rolling direction is determined exactly and can be
 * read off directly, without the weighted heuristic and nullspace projection
 * that the (redundant) force distribution requires.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] xd_drv A matrix with two rows and @p num_drv columns where the
 *                   rows represent the linear velocity components of the
 *                   drive's attachment point (to the platform) in the
 *                   longitudinal and transverse direction, respectively. The
 *                   coordinates are expressed in the individual drives' pivot
 *                   frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and must be provided in column-major order.
 * @param[out] dst An array with @p num_drv elements that represent the pivot
 *                 alignment distance of each drive with respect to its
 *                 commanded attachment velocity. The array is arranged as @f$
 *                 \begin{bmatrix}
 *                   dst_1 & \ldots & dst_n
 *                 \end{bmatrix}@f$.
 * @param[in] inc_dst Increment of the @p dst array.
 */
void hddc2b_drv_vel_algn_dst(
        int num_drv,
        const double *xd_drv,
        double *dst,
        int inc_dst);


/**
 * Compute the scrub angle of each drive under a platform twist. The scrub
 * angle is the angle between the drive's rolling direction (the pivot frame's
 * x-axis) and the velocity that the drive's axle centre would have if the
 * pivot did not rotate relative to the platform, i.e. if the castor turned
 * with the platform at @f$\omega_p@f$:
 * @f[
 *   dst_i = \operatorname{atan2}(\dot{X}_{i,y} - l_i \omega_p, \dot{X}_{i,x})
 * @f]
 * where @f$l_i@f$ is the castor offset. The scrub angle is zero exactly when
 * the drive needs no pivot motion to follow the twist; that is the
 * steady-state alignment, also while the platform rotates. In contrast to
 * @ref hddc2b_drv_vel_algn_dst the angle is defined with respect to the axle
 * centre, not the pivot point.
 *
 * A drive that rolls exactly backwards has a scrub angle of @f$\pm\pi@f$ where
 * the sign is numerically arbitrary. This function always returns @f$+\pi@f$
 * so that all such drives are steered in the same, counter-clockwise,
 * direction. For a vanishing axle velocity the scrub angle is zero.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] cstr_off The castor offset, an array with @p num_drv elements that
 *                     represent the (scalar) distance between the wheel axle
 *                     and the drive's attachment point (to the platform). The
 *                     array is arranged as @f$
 *                     \begin{bmatrix}
 *                       co_1 & \ldots & co_n
 *                     \end{bmatrix}@f$.
 * @param[in] omega_pltf The platform's angular velocity @f$\omega_p@f$.
 * @param[in] xd_drv A matrix with two rows and @p num_drv columns where the
 *                   rows represent the linear velocity components of the
 *                   drive's attachment point (to the platform) in the
 *                   longitudinal and transverse direction, respectively. The
 *                   coordinates are expressed in the individual drives' pivot
 *                   frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and must be provided in column-major order.
 * @param[out] dst An array with @p num_drv elements that represent the scrub
 *                 angle of each drive. The array is arranged as @f$
 *                 \begin{bmatrix}
 *                   dst_1 & \ldots & dst_n
 *                 \end{bmatrix}@f$.
 * @param[in] inc_dst Increment of the @p dst array.
 * @param[out] spd An array with @p num_drv elements that represent the speed
 *                 @f$v_i@f$ of each drive's axle centre, i.e. the magnitude of
 *                 the velocity whose direction @p dst is. The array is
 *                 arranged as @f$
 *                 \begin{bmatrix}
 *                   v_1 & \ldots & v_n
 *                 \end{bmatrix}@f$.
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
 * Compute the transverse pivot velocity that each drive requires to remove its
 * scrub angle. A castor rotates at @f$\dot{q}_i = \dot{X}_{i,y} / l_i@f$ in
 * the world when its pivot point moves transversely at @f$\dot{X}_{i,y}@f$, so
 * the reference
 * @f[
 *   \dot{\bar{X}}_{i,y} = l_i \left(
 *     \operatorname{clip}\left(
 *       \min\left(\frac{1}{\tau}, \frac{v_i}{l_i}\right) dst_i,
 *       \pm\dot{q}_{max}\right)
 *     + \omega_p \right)
 * @f]
 * turns the castor with the platform and additionally drives the scrub angle
 * to zero, at most at the rate @f$\dot{q}_{max}@f$ relative to the platform.
 *
 * A castor aligns by rolling: left to itself a trailing castor decays its
 * scrub angle with the time constant @f$l_i / v_i@f$, so at the speed
 * @f$v_i@f$ the drive currently rolls at that is as fast as the wheels can
 * steer it without fighting the platform motion. The effective time constant
 * is therefore the slower of @f$\tau@f$ and @f$l_i / v_i@f$, which also means
 * that a drive at a standstill is not asked to swing at all.
 *
 * @param[in] num_drv The number of drives that this function is applied to.
 * @param[in] cstr_off The castor offset, an array with @p num_drv elements that
 *                     represent the (scalar) distance between the wheel axle
 *                     and the drive's attachment point (to the platform). The
 *                     array is arranged as @f$
 *                     \begin{bmatrix}
 *                       co_1 & \ldots & co_n
 *                     \end{bmatrix}@f$.
 * @param[in] tau The shortest time constant @f$\tau@f$ within which to remove
 *                the scrub angle. The time constant must be positive.
 * @param[in] qd_max The maximum castor rate @f$\dot{q}_{max}@f$ with respect to
 *                   the platform. The rate must be non-negative.
 * @param[in] omega_pltf The platform's angular velocity @f$\omega_p@f$.
 * @param[in] dst An array with @p num_drv elements that represent the scrub
 *                angle of each drive, as computed by
 *                @ref hddc2b_drv_vel_scrb_dst. The array is arranged as @f$
 *                \begin{bmatrix}
 *                  dst_1 & \ldots & dst_n
 *                \end{bmatrix}@f$.
 * @param[in] inc_dst Increment of the @p dst array.
 * @param[in] spd An array with @p num_drv elements that represent the speed
 *                @f$v_i@f$ of each drive's axle centre, as computed by
 *                @ref hddc2b_drv_vel_scrb_dst. The array is arranged as @f$
 *                \begin{bmatrix}
 *                  v_1 & \ldots & v_n
 *                \end{bmatrix}@f$.
 * @param[in] inc_spd Increment of the @p spd array.
 * @param[out] xd_ref An array with @p num_drv elements that represent the
 *                    transverse velocity reference
 *                    @f$\bar{\dot{X}}_{i,y}@f$ of each drive's attachment
 *                    point. The coordinates are expressed in the individual
 *                    drives' pivot frames. The array is arranged as @f$
 *                    \begin{bmatrix}
 *                      \bar{\dot{X}}_{1,y} & \ldots & \bar{\dot{X}}_{n,y}
 *                    \end{bmatrix}@f$.
 * @param[in] inc_ref Increment of the @p xd_ref array. An increment of two
 *                    writes the transverse row of a drive velocity matrix.
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
