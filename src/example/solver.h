#ifndef HDDC2B_SOLVER_H
#define HDDC2B_SOLVER_H


#ifdef __cplusplus
extern "C" {
#endif


/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is either **singular** (less than one drive unit) or **redundant**
 * (two or more drive units), compute the weight decomposition internally and
 * include a reference drive force in the nullspace.
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
 *                   @f] where @f$w_{p,yx} = w_{p,xy}@f$,
 *                   @f$w_{p,mx} = w_{p,xm}@f$ and @f$w_{p,my} = w_{p,ym}@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
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
 *                      force reference components that are projected into the
 *                      nullspace of the platform task. The forces' coordinates
 *                      are expressed in the individual drives' pivot frames.
 *                      The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and will be provided in column-major order.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void hddc2b_example_frc(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf,
        const double *f_pltf,
        const double *w_drv,
        const double *f_drv_ref,
        double *f_drv);


/**
 * Find a solution to the velocity composition problem for a situation where the
 * platform is either **singular** (less than one drive unit) or **redundant**
 * (two or more drive units).
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
 * @param[in] xd_drv The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                   @p num_drv columns where the rows represent the linear
 *                   velocity components of each drive. The velocities'
 *                   reference point is the origin of the individual drives'
 *                   pivot frames and the coordinates are expressed in these
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
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
 *                       @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                       provided in column-major order.
 * @param[out] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                     that represent the linear and angular velocity of the
 *                     platform. The linear velocitie's reference point is the
 *                     platform's origin. The coordinates are expressed in the
 *                     platform frame. The vector is arranged as @f$
 *                     \begin{bmatrix}
 *                       v_{p,x} & v_{p,y} & \omega_p
 *                     \end{bmatrix}@f$.
 */
void hddc2b_example_vel(
        int num_drv,
        double eps,
        const double *g,
        const double *w_drv_sqrt,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf);


/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is either **singular** (less than one drive unit) or **redundant**
 * (two or more drive units).
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
 * @param[in] w_pltf_sqrt The square root of the platform's weight matrix
 *                        @f$\vect{W}_p^{\frac{1}{2}}@f$, a positive-definite
 *                        @f$3 \times 3@f$ matrix. The weight is measured in the
 *                        platform's origin and its coordinates are expressed in
 *                        the platform frame. The matrix is arranged as @f[
 *                        \begin{bmatrix}
 *                          w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                          w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                          w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                        \end{bmatrix}
 *                        @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                        @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                        @f$w_{p,my}' = w_{p,ym}'@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[in] w_drv_inv_sqrt An array that consists of @p num_drv
 *                           positive-definite @f$2 \times 2@f$ weight matrices'
 *                           square roots @f$\vect{W}_d^{-\frac{1}{2}}@f$, each
 *                           measured in the respective drive's attachment point
 *                           and its coordinates expressed in the drive's
 *                           attachment frame. The array is arranged as @f[
 *                           \begin{bmatrix}
 *                             w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                             w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                             w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                             w_{1,yy}' & \ldots & w_{n,yy}'
 *                           \end{bmatrix}
 *                           @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                           provided in column-major order.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void ex_frc_pltf_to_pvt_redu_sing_pinv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf_sqrt,
        const double *f_pltf,
        const double *w_drv_inv_sqrt,
        double *f_drv);


/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is either **singular** (less than one drive unit) or **redundant**
 * (two or more drive units) and include a reference drive force in the
 * nullspace.
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
 * @param[in] w_pltf_sqrt The square root of the platform's weight matrix
 *                        @f$\vect{W}_p^{\frac{1}{2}}@f$, a positive-definite
 *                        @f$3 \times 3@f$ matrix. The weight is measured in the
 *                        platform's origin and its coordinates are expressed in
 *                        the platform frame. The matrix is arranged as @f[
 *                        \begin{bmatrix}
 *                          w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                          w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                          w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                        \end{bmatrix}
 *                        @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                        @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                        @f$w_{p,my}' = w_{p,ym}'@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[in] w_drv_inv_sqrt An array that consists of @p num_drv
 *                           positive-definite @f$2 \times 2@f$ weight matrices'
 *                           square roots @f$\vect{W}_d^{-\frac{1}{2}}@f$, each
 *                           measured in the respective drive's attachment point
 *                           and its coordinates expressed in the drive's
 *                           attachment frame. The array is arranged as @f[
 *                           \begin{bmatrix}
 *                             w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                             w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                             w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                             w_{1,yy}' & \ldots & w_{n,yy}'
 *                           \end{bmatrix}
 *                           @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                           provided in column-major order.
 * @param[in] f_drv_ref The matrix @f$\bar{\vect{F}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent the linear
 *                      force reference components that are projected into the
 *                      nullspace of the platform task. The forces' coordinates
 *                      are expressed in the individual drives' pivot frames.
 *                      The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and will be provided in column-major order.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void ex_frc_pltf_to_pvt_redu_sing_ref_pinv(
        int num_drv,
        double eps,
        const double *g,
        const double *w_pltf_sqrt,
        const double *f_pltf,
        const double *w_drv_inv_sqrt,
        const double *f_drv_ref,
        double *f_drv);


/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is **singular** (less than one drive unit).
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] lambda The scalar @f$\lambda@f$ that determines the "amount" of
 *                   damping in damped least squares.
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
 * @param[in] w_pltf_sqrt The square root of the platform's weight matrix
 *                        @f$\vect{W}_p^{\frac{1}{2}}@f$, a positive-definite
 *                        @f$3 \times 3@f$ matrix. The weight is measured in the
 *                        platform's origin and its coordinates are expressed in
 *                        the platform frame. The matrix is arranged as @f[
 *                        \begin{bmatrix}
 *                          w_{p,xx}' & w_{p,yx}' & w_{p,mx}' \\
 *                          w_{p,xy}' & w_{p,yy}' & w_{p,my}' \\
 *                          w_{p,xm}' & w_{p,ym}' & w_{p,mm}'
 *                        \end{bmatrix}
 *                        @f] where @f$w_{p,yx}' = w_{p,xy}'@f$,
 *                        @f$w_{p,mx}' = w_{p,xm}'@f$ and
 *                        @f$w_{p,my}' = w_{p,ym}'@f$.
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void ex_frc_pltf_to_pvt_sing_dls(
        int num_drv,
        double eps,
        double lambda,
        const double *g,
        const double *w_pltf_sqrt,
        const double *f_pltf,
        double *f_drv);


/**
 * Find a solution to the force distribution problem for a situation where the
 * platform is **redundant** (more than one drive unit).
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
 * @param[in] f_pltf The vector @f$\vect{F}_p@f$ with three elements that
 *                   represent a force and torque on the platform. The torque's
 *                   reference point is the platform's origin. The coordinates
 *                   are expressed in the platform frame. The vector is arranged
 *                   as @f$
 *                   \begin{bmatrix}
 *                     f_{p,x} & f_{p,y} & m_p
 *                   \end{bmatrix}@f$.
 * @param[in] w_drv_inv_sqrt An array that consists of @p num_drv
 *                           positive-definite @f$2 \times 2@f$ weight matrices'
 *                           square roots @f$\vect{W}_d^{-\frac{1}{2}}@f$, each
 *                           measured in the respective drive's attachment point
 *                           and its coordinates expressed in the drive's
 *                           attachment frame. The array is arranged as @f[
 *                           \begin{bmatrix}
 *                             w_{1,xx}' & \ldots & w_{n,xx}' \\
 *                             w_{1,yx}' & \ldots & w_{n,yx}' \\
 *                             w_{1,xy}' & \ldots & w_{n,xy}' \\
 *                             w_{1,yy}' & \ldots & w_{n,yy}'
 *                           \end{bmatrix}
 *                           @f] where @f$w_{i,yx}' = w_{i,xy}'@f$ and will be
 *                           provided in column-major order.
 * @param[in] f_drv_ref The matrix @f$\bar{\vect{F}}_d@f$ with two rows and
 *                      @p num_drv columns where the rows represent the linear
 *                      force reference components that are projected into the
 *                      nullspace of the platform task. The forces' coordinates
 *                      are expressed in the individual drives' pivot frames.
 *                      The matrix is arranged as @f[
 *                      \begin{bmatrix}
 *                        \bar{f}_{1,x} & \ldots & \bar{f}_{n,x} \\
 *                        \bar{f}_{1,y} & \ldots & \bar{f}_{n,y}
 *                      \end{bmatrix}
 *                      @f] and will be provided in column-major order.
 * @param[out] f_drv The matrix @f$\vect{F}_d@f$ with two rows and @p num_drv
 *                   columns where the rows represent the linear force
 *                   components that each drive exerts on the platform. The
 *                   forces' coordinates are expressed in the individual drives'
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     f_{1,x} & \ldots & f_{n,x} \\
 *                     f_{1,y} & \ldots & f_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 */
void ex_frc_pltf_to_pvt_redu_ref_pinv(
        int num_drv,
        double eps,
        const double *g,
        const double *f_pltf,
        const double *w_drv_inv_sqrt,
        const double *f_drv_ref,
        double *f_drv);


/**
 * Find a solution to the velocity composition problem for a situation where the
 * platform is **singular** (less than one drive unit).
 *
 * @param[in] num_drv The number of drives that the platform consists of.
 * @param[in] eps The scalar @f$\epsilon@f$ that determines when to compute the
 *                inverse.
 * @param[in] lambda The scalar @f$\lambda@f$ that determines the "amount" of
 *                   damping in damped least squares.
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
 * @param[in] xd_drv The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                   @p num_drv columns where the rows represent the linear
 *                   velocity components of each drive. The velocities'
 *                   reference point is the origin of the individual drives'
 *                   pivot frames and the coordinates are expressed in these
 *                   pivot frames. The matrix is arranged as @f[
 *                   \begin{bmatrix}
 *                     \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                     \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                   \end{bmatrix}
 *                   @f] and will be provided in column-major order.
 * @param[out] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                     that represent the linear and angular velocity of the
 *                     platform. The linear velocitie's reference point is the
 *                     platform's origin. The coordinates are expressed in the
 *                     platform frame. The vector is arranged as @f$
 *                     \begin{bmatrix}
 *                       v_{p,x} & v_{p,y} & \omega_p
 *                     \end{bmatrix}@f$.
 */
void ex_vel_pvt_to_pltf_sing_dls(
        int num_drv,
        double eps,
        double lambda,
        const double *g,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf);


/**
 * Distribute a platform twist to the pivot velocities. The map is unique
 * (the kinematic dual of force composition).
 *
 * @param[in] num_drv The number of drives that the platform consists of.
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
 * @param[in] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                    that represent the linear and angular velocity of the
 *                    platform. The linear velocity's reference point is the
 *                    platform's origin. The coordinates are expressed in the
 *                    platform frame. The vector is arranged as @f$
 *                    \begin{bmatrix}
 *                      v_{p,x} & v_{p,y} & \omega_p
 *                    \end{bmatrix}@f$.
 * @param[out] xd_drv The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                    @p num_drv columns where the rows represent the linear
 *                    velocity components of each drive. The velocities'
 *                    reference point is the origin of the individual drives'
 *                    pivot frames and the coordinates are expressed in these
 *                    pivot frames. The matrix is arranged as @f[
 *                    \begin{bmatrix}
 *                      \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                      \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                    \end{bmatrix}
 *                    @f] and will be provided in column-major order.
 */
void ex_vel_pltf_to_pvt(
        int num_drv,
        const double *g,
        const double *xd_pltf,
        double *xd_drv);


/**
 * Distribute a platform twist to the wheel hub speeds. Castor alignment is
 * a lower-weighted task that relaxes the twist, since the unique
 * platform-to-pivot map leaves no nullspace for it. The hub speeds are
 * limited by scaling the relaxed twist uniformly.
 *
 * @param[in] num_drv The number of drives that the platform consists of.
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
 * @param[in] whl_dia An array with two rows and @p num_drv columns where the rows
 *                    contain the diameter of the right and left wheel,
 *                    respectively. The array must be provided in column-major
 *                    order.
 * @param[in] whl_dst An array with @p num_drv elements that represent the
 *                    distance between each wheel and the central point between the
 *                    wheels.
 * @param[in] cstr_off The castor offset, an array with @p num_drv elements that
 *                     represent the distance between the wheel axle and the
 *                     drive's attachment point.
 * @param[in] w_pltf The platform's tracking weight matrix @f$\vect{W}_p@f$, a
 *                   positive-definite @f$3 \times 3@f$ matrix. The weight is
 *                   measured in the platform's origin and its coordinates are
 *                   expressed in the platform frame. The matrix is arranged as
 *                   @f[
 *                   \begin{bmatrix}
 *                     w_{p,xx} & w_{p,yx} & w_{p,mx} \\
 *                     w_{p,xy} & w_{p,yy} & w_{p,my} \\
 *                     w_{p,xm} & w_{p,ym} & w_{p,mm}
 *                   \end{bmatrix}
 *                   @f] where @f$w_{p,yx} = w_{p,xy}@f$,
 *                   @f$w_{p,mx} = w_{p,xm}@f$ and @f$w_{p,my} = w_{p,ym}@f$.
 *                   The ratio between this weight and the alignment weights
 *                   determines how much platform velocity is given up to align
 *                   the castors.
 * @param[in] w_algn An array with @p num_drv elements that represent a
 *                   non-negative weight @f$w_{a,i}@f$ for each drive's castor
 *                   alignment task. The array is arranged as @f$
 *                   \begin{bmatrix}
 *                     w_{a,1} & \ldots & w_{a,n}
 *                   \end{bmatrix}@f$.
 * @param[in] tau The time constant @f$\tau@f$ within which to remove the
 *                drives' scrub angle. The time constant must be positive.
 * @param[in] qd_max The maximum castor rate @f$\dot{q}_{max}@f$ with respect to
 *                   the platform. The rate must be non-negative.
 * @param[in] omega_max The maximum admissible hub speed @f$\omega_{max}@f$. The
 *                      speed must be positive.
 * @param[in] xd_pltf The vector @f$\dot{\vect{X}}_p@f$ with three elements
 *                    that represent the linear and angular velocity of the
 *                    platform. The linear velocity's reference point is the
 *                    platform's origin. The coordinates are expressed in the
 *                    platform frame. The vector is arranged as @f$
 *                    \begin{bmatrix}
 *                      v_{p,x} & v_{p,y} & \omega_p
 *                    \end{bmatrix}@f$.
 * @param[out] xd_pltf_eff The vector @f$\dot{\vect{X}}_{eff}@f$ with three
 *                         elements that represent the effective linear and
 *                         angular velocity of the platform, i.e. after the
 *                         castor alignment has relaxed and the hub speed limit
 *                         has scaled the commanded twist. The vector is
 *                         arranged in the same way as @p xd_pltf.
 * @param[out] xd_drv The matrix @f$\dot{\vect{X}}_d@f$ with two rows and
 *                    @p num_drv columns where the rows represent the linear
 *                    velocity components of each drive. The velocities'
 *                    reference point is the origin of the individual drives'
 *                    pivot frames and the coordinates are expressed in these
 *                    pivot frames. The matrix is arranged as @f[
 *                    \begin{bmatrix}
 *                      \dot{X}_{1,x} & \ldots & \dot{X}_{n,x} \\
 *                      \dot{X}_{1,y} & \ldots & \dot{X}_{n,y}
 *                    \end{bmatrix}
 *                    @f] and will be provided in column-major order.
 * @param[out] omega_hub The matrix with two rows and @p num_drv columns where the
 *                       rows contain the angular velocity around the right and
 *                       left wheel axle, respectively. The matrix is arranged in
 *                       column-major order.
 */
void hddc2b_example_vel_dist(
        int num_drv,
        const double *g,
        const double *whl_dia,
        const double *whl_dst,
        const double *cstr_off,
        const double *w_pltf,
        const double *w_algn,
        double tau,
        double qd_max,
        double omega_max,
        const double *xd_pltf,
        double *xd_pltf_eff,
        double *xd_drv,
        double *omega_hub);

#ifdef __cplusplus
}
#endif

#endif
