// SPDX-License-Identifier: LGPL-3.0
#ifndef HDDC2B_FUNCTIONS_SOLVER_H
#define HDDC2B_FUNCTIONS_SOLVER_H


#ifdef __cplusplus
extern "C" {
#endif

/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is either **singular** (less than one drive unit) or
 * **redundant** (two or more drive units), compute the weight decomposition
 * internally and include a reference drive force in the nullspace:
 *
 * @f[
 *   \vect{F}_{prim} &= \vect{W}_d^{-\frac{1}{2}}
 *     \left(\vect{V} \vect{S}^+ \vect{U}^T\right)
 *     \vect{W}_p^{\frac{1}{2}} \vect{F}_p \\
 *   \vect{F}_{scnd} &= \bar{\vect{F}}_d - \vect{W}_d^{-\frac{1}{2}}
 *     \left(\vect{V} \vect{S}^+ \vect{U}^T\right)
 *     \vect{W}_p^{\frac{1}{2}} \vect{G} \bar{\vect{F}}_d
 * @f]
 * where @f$\vect{U}, \vect{S}, \vect{V}^T@f$ is the singular value
 * decomposition of @f$\vect{W}_p^{\frac{1}{2}} \vect{G}
 * \vect{W}_d^{-\frac{1}{2}}@f$ and @f$\vect{S}^+@f$ its pseudoinverse.
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] g The force composition matrix @f$\vect{G}@f$ with three rows and
 *              @f$2 \times {}@f$ @p num_drv columns, measured in the platform's
 *              origin and its coordinates expressed in the platform's frame.
 *              The matrix is arranged as @f[
 *              \begin{bmatrix}
 *                \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *              \end{bmatrix}
 *              @f] and must be provided in column-major order. Here,
 *              @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the platform
 *              forces and torque, respectively. @f$f_{i,x}@f$ and @f$f_{i,y}@f$
 *              are the drive's forces.
 * @param[in] w_pltf The platform's weight matrix @f$\vect{W}_p@f$, a
 *                   positive-definite @f$3 \times 3@f$ matrix. The weight is
 *                   measured in the platform's origin and its coordinates are
 *                   expressed in the platform frame. The matrix is arranged as
 *                   @f[
 *                   \begin{bmatrix}
 *                     w_{p,xx} & w_{p,yx} & w_{p,mx} \\
 *                     w_{p,xy} & w_{p,yy} & w_{p,my} \\
 *                     w_{p,xm} & w_{p,ym} & w_{p,mm}
 *                   \end{bmatrix}
 *                   @f] where
 *                   @f$w_{p,yx} = w_{p,xy}@f$, @f$w_{p,mx} = w_{p,xm}@f$
 *                   and @f$w_{p,my} = w_{p,ym}@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The
 *                   torque's reference point is the platform's origin. The
 *                   coordinates are expressed in the platform frame. The
 *                   vector is arranged as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[in] w_drv An array that consists of @p num_drv positive-definite
 *                  @f$2 \times 2@f$ weight matrices @f$\vect{W}_d@f$, each
 *                  measured in the respective drive's attachment point and its
 *                  coordinates expressed in the drive's attachment frame. The
 *                  array is arranged as @f[
 *                  \begin{bmatrix}
 *                    w_{1,xx} & \ldots & w_{n,xx} \\
 *                    w_{1,yx} & \ldots & w_{n,yx} \\
 *                    w_{1,xy} & \ldots & w_{n,xy} \\
 *                    w_{1,yy} & \ldots & w_{n,yy}
 *                  \end{bmatrix}
 *                  @f] where @f$w_{i,yx} = w_{i,xy}@f$ and must be provided in
 *                  column-major order.
 * @param[in] f_drv_ref The matrix @f$\bar{\vect{F}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent the linear
 *                      force reference components in the longitudinal and
 *                      transverse direction, respectively, that will be
 *                      projected into the nullspace of the platform task. The
 *                      forces' coordinates are expressed in the individual
 *                      drives' pivot frames. The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and must be provided in column-major order.
 * @param[out] f_drv_prim The matrix @f$\vect{F}_{prim}@f$ with two rows and
 *                        @p num_drv columns that represents the primary-task
 *                        drive forces resulting from distributing the
 *                        platform-level force @p f_pltf. The matrix is
 *                        arranged as @f[
 *                        \begin{bmatrix}
 *                          f_{1,x} & \ldots & f_{n,x} \\
 *                          f_{1,y} & \ldots & f_{n,y}
 *                        \end{bmatrix}
 *                        @f] and will be provided in column-major order.
 * @param[out] f_drv_scnd The matrix @f$\vect{F}_{scnd}@f$ with two rows and
 *                        @p num_drv columns that represents the part of
 *                        @p f_drv_ref projected into the nullspace of the
 *                        platform-level task. The matrix is arranged the same
 *                        as @p f_drv_prim and will be provided in column-major
 *                        order. The combined drive force is
 *                        @f$\vect{F}_d = \vect{F}_{prim} + \vect{F}_{scnd}@f$.
 */
void hddc2b_pltf_frc_pltf_to_drv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv_prim,
        double *f_drv_scnd);


/**
 * The kinematic dual of hddc2b_pltf_frc_pltf_to_drv() for a situation where
 * the platform is either **singular** (less than one drive unit) or
 * **redundant** (two or more drive units): the primary task is the unique
 * velocity distribution for the commanded platform twist, and the secondary
 * task keeps only the part of a drive-space reference velocity that does not
 * induce any platform twist under the same weighted velocity-composition
 * model (with weights @p w_drv_sqrt and @p w_pltf_inv_sqrt) used to compose a
 * platform velocity from wheel velocities:
 *
 * @f[
 *   \dot{\vect{X}}_{prim} &= \vect{G}^T \dot{\vect{X}}_p \\
 *   \dot{\vect{X}}_{scnd} &= \dot{\vect{X}}_{ref}
 *     - \vect{G}^T \vect{W}_p^{-\frac{1}{2}}
 *       \left(\vect{U} \vect{S}^+ \vect{V}^T\right)
 *       \vect{W}_d^{\frac{1}{2}} \dot{\vect{X}}_{ref}
 * @f]
 * where @f$\vect{U}, \vect{S}, \vect{V}^T@f$ is the singular value
 * decomposition of @f$\vect{W}_p^{-\frac{1}{2}} \vect{G}
 * \vect{W}_d^{\frac{1}{2}}@f$ and @f$\vect{S}^+@f$ its pseudoinverse.
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] g The force composition matrix @f$\vect{G}@f$ with three rows and
 *              @f$2 \times {}@f$ @p num_drv columns, measured in the platform's
 *              origin and its coordinates expressed in the platform's frame.
 *              The matrix is arranged as @f[
 *              \begin{bmatrix}
 *                \frac{\partial{f_{p,x}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,x}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{f_{p,y}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{f_{p,y}}}{\partial{f_{n,y}}} \\
 *                \frac{\partial{m_{p,z}}}{\partial{f_{1,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{1,y}}} & \ldots
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,x}}}
 *                  & \frac{\partial{m_{p,z}}}{\partial{f_{n,y}}}
 *              \end{bmatrix}
 *              @f] and must be provided in column-major order. Here,
 *              @f$f_{p,x}@f$, @f$f_{p,y}@f$ and @f$m_{p,z}@f$ are the platform
 *              forces and torque, respectively. @f$f_{i,x}@f$ and @f$f_{i,y}@f$
 *              are the drive's forces.
 * @param[in] w_drv_sqrt An array that consists of @p num_drv positive-definite
 *                       @f$2 \times 2@f$ weight matrices' square roots
 *                       @f$\vect{W}_d^{\frac{1}{2}}@f$, each measured in the
 *                       respective drive's attachment point and its coordinates
 *                       expressed in the drive's attachment frame. The array is
 *                       arranged as @f[
 *                       \begin{bmatrix}
 *                         w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                         w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                         w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                         w_{1,yy}' & \ldots & w_{n,yy}'
 *                       \end{bmatrix}
 *                       @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and must be
 *                       provided in column-major order.
 * @param[in] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                    that represent the linear and angular velocity of the
 *                    platform. The linear velocity's reference point is the
 *                    platform's origin. The coordinates are expressed in the
 *                    platform frame. The vector is arranged as @f$
 *                    \begin{bmatrix}
 *                      v_{p,x} & v_{p,y} & \omega_p
 *                    \end{bmatrix}@f$.
 * @param[in] w_pltf_inv_sqrt The inverse square root of the platform's weight
 *                            matrix @f$\vect{W}_p^{-\frac{1}{2}}@f$, a
 *                            positive-definite @f$3 \times 3@f$ matrix. The
 *                            weight is measured in the platform's origin and
 *                            its coordinates are expressed in the platform
 *                            frame. The matrix is arranged as @f[
 *                            \begin{bmatrix}
 *                              w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                              w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                              w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                            \end{bmatrix}
 *                            @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                            @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                            @f$w_{p,my}' = w_{p,ym}'@f$.
 * @param[in] xd_drv_ref The matrix @f$\bar{\dot{\vect{X}}}_d@f$ with two rows
 *                       and @p num_drv columns where the rows represent the
 *                       linear velocity reference components in the
 *                       longitudinal and transverse direction, respectively,
 *                       of which the part that does not induce a platform
 *                       twist will be returned in @p xd_drv_scnd. The
 *                       velocities' coordinates are expressed in the
 *                       individual drives' pivot frames. The matrix is
 *                       arranged as @f[
 *                       \begin{bmatrix}
 *                         \bar{\dot{X}}_{1,x} & \ldots & \bar{\dot{X}}_{n,x} \\
 *                         \bar{\dot{X}}_{1,y} & \ldots & \bar{\dot{X}}_{n,y}
 *                       \end{bmatrix}
 *                       @f] and must be provided in column-major order.
 * @param[out] xd_drv_prim The matrix @f$\dot{\vect{X}}_{prim}@f$ with two rows
 *                         and @p num_drv columns that represents the
 *                         primary-task drive velocities resulting from
 *                         distributing the platform twist @p xd_pltf. The
 *                         matrix is arranged as @f[
 *                         \begin{bmatrix}
 *                           \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                           \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                         \end{bmatrix}
 *                         @f] and will be provided in column-major order.
 * @param[out] xd_drv_scnd The matrix @f$\dot{\vect{X}}_{scnd}@f$ with two rows
 *                         and @p num_drv columns that represents the part of
 *                         @p xd_drv_ref that does not induce a platform twist.
 *                         The matrix is arranged the same as @p xd_drv_prim
 *                         and will be provided in column-major order.
 */
void hddc2b_pltf_vel_pltf_to_drv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_drv_sqrt,
        const double *xd_pltf,
        const double *w_pltf_inv_sqrt,
        const double *xd_drv_ref,
        double *xd_drv_prim,
        double *xd_drv_scnd);


#ifdef __cplusplus
}
#endif

#endif
