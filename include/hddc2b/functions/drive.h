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


#ifdef __cplusplus
}
#endif

#endif
